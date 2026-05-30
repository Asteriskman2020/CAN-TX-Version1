// ============================================================================
//  Wemos D1 Mini ESP32 — CAN sniffer + ESP-NOW bridge + Garage Opener
//  ----------------------------------------------------------------------------
//  Passive only: CAN driver in LISTEN_ONLY mode — never injects ACK or any
//  dominant bit onto the bus.
//
//    - CAN RX via TJA1050 @ 500 kbps, accept-all filter
//    - ESP-NOW broadcast forwarding of every received CAN frame
//    - Garage opener (from idealabkmutt/ODB2/FORD_Benz_garage_opener_2PID):
//        * Benz S320 0x3F6 bit 12  = "Speak" button on steering wheel
//        * Ford Ranger 0x09A bit 56 = Cruise main switch
//        * Either button held >= 3 s -> fires local relay for 1 s
//
//  Pins:
//        GARAGE_RELAY = GPIO5   (D1 silkscreen)
//        LED          = GPIO2   (D4, active-LOW)
//        CAN_TX       = GPIO21  (to TJA1050 CTX, still wired but unused in TX)
//        CAN_RX       = GPIO22  (to TJA1050 CRX)
//        BTN_BOOT     = GPIO0
//
//  Serial REPL @ 115200:
//        help / stats / clear
//        enow on|off|toggle      ESP-NOW relay
//        garage                  manually pulse the relay
//
//  BTN:  short tap = ESP-NOW toggle
//        long press >=2s = manual garage trigger
//
//  Hardware: Wemos D1 Mini ESP32 (WROOM-32) + TJA1050 (120Ω termination)
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"

// ---- Pins ----
#define GARAGE_RELAY_PIN  5
#define LED_PIN           2    // active-LOW
#define CAN_TX           21
#define CAN_RX           22
#define BTN_BOOT          0

// ---- Garage opener tuning ----
#define BENZ_SWC_CAN_ID         0x3F6   // 1014 - VCLEFT_switchStatus (Speak btn bit 12)
#define FORD_CLU11_CAN_ID       0x09A   // 154  - CLU11 (Cruise btn bit 56)
#define GARAGE_HOLD_MS          3000
#define GARAGE_PULSE_MS         1000
#define GARAGE_REARM_MS         2000

// ---- ESP-NOW payload ----
struct __attribute__((packed)) CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

// ---- Globals ----
static uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint32_t rxCount = 0, enowTxCount = 0, enowTxFail = 0;
static uint32_t garageTriggers = 0;
static bool espNowEnabled = true;

// Garage state machine
static bool        btnHeld          = false;
static uint32_t    btnPressStartMs  = 0;
static const char* btnHeldSource    = "-";
static uint32_t    garagePulseStart = 0;
static uint32_t    lastTriggerMs    = 0;

// BOOT button
static int      lastBtnState   = HIGH;
static uint32_t pressMs        = 0;
static bool     handledPress   = false;
static uint32_t lastLedToggle  = 0;
static uint32_t lastActivityMs = 0;
static bool     ledState       = false;

static String  serialBuf;
static const size_t SERIAL_BUF_MAX = 64;

// ---- LED (active-LOW) ----
static void setLED(bool on) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);
    ledState = on;
}

// ---- Print helpers ----
static void printFrame(uint32_t id, uint8_t dlc, const uint8_t* data, bool ext) {
    Serial.printf("[RX] 0x%lX%s dlc=%u", (unsigned long)id, ext ? "X" : "", dlc);
    for (int i = 0; i < dlc && i < 8; i++) Serial.printf(" %02X", data[i]);
    Serial.println();
}

// ---- CAN (LISTEN_ONLY: silent on the bus) ----
static void initCAN() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
    Serial.println("[CAN] Started @ 500 kbps, LISTEN_ONLY (TX=GPIO21 RX=GPIO22)");
}

// ---- ESP-NOW ----
static void initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] init FAILED");
        return;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.printf("[ESP-NOW] Ready - MAC: %s\n", WiFi.macAddress().c_str());
}

// ---- Garage opener ----
// Bit numbering: raw_ul = d[0..7] little-endian -> bit N = byte (N/8) bit (N%8)
//   Benz speak  (bit 12) -> data[1] >> 4 & 1
//   Ford cruise (bit 56) -> data[7] & 1
static void watchGarageButton(const twai_message_t& msg) {
    bool pressed = false;
    const char* source = nullptr;

    if (msg.identifier == BENZ_SWC_CAN_ID && msg.data_length_code >= 2) {
        pressed = (msg.data[1] >> 4) & 0x01;
        source = "BENZ_SPEAK";
    } else if (msg.identifier == FORD_CLU11_CAN_ID && msg.data_length_code >= 8) {
        pressed = msg.data[7] & 0x01;
        source = "FORD_CRUISE";
    } else {
        return;
    }

    uint32_t now = millis();
    if (pressed && !btnHeld) {
        btnHeld = true;
        btnPressStartMs = now;
        btnHeldSource = source;
        Serial.printf("[GARAGE] %s pressed - holding...\n", source);
    } else if (!pressed && btnHeld) {
        btnHeld = false;
        btnPressStartMs = 0;
        btnHeldSource = "-";
    }
}

static void garageStart(const char* reason) {
    if (garagePulseStart != 0) return;
    if (millis() - lastTriggerMs < GARAGE_REARM_MS && lastTriggerMs) return;
    Serial.printf("[GARAGE] OPEN (%s)\n", reason);
    digitalWrite(GARAGE_RELAY_PIN, HIGH);
    setLED(true);
    garagePulseStart = millis();
    garageTriggers++;
}

static void garageService() {
    if (!garagePulseStart) return;
    if (millis() - garagePulseStart >= GARAGE_PULSE_MS) {
        digitalWrite(GARAGE_RELAY_PIN, LOW);
        setLED(false);
        lastTriggerMs = millis();
        garagePulseStart = 0;
        Serial.println("[GARAGE] released");
    }
}

// ---- Serial REPL ----
static void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  help                     this help"));
    Serial.println(F("  enow on|off|toggle       ESP-NOW relay"));
    Serial.println(F("  garage                   manually pulse the relay"));
    Serial.println(F("  stats                    counters + flags"));
    Serial.println(F("  clear                    reset counters"));
}

static void printStats() {
    Serial.printf("[STAT] rx=%lu enowtx=%lu enowfail=%lu gates=%lu"
                  " | enow=%s held=%s up=%lums\n",
        (unsigned long)rxCount, (unsigned long)enowTxCount, (unsigned long)enowTxFail,
        (unsigned long)garageTriggers,
        espNowEnabled ? "ON" : "OFF",
        btnHeld ? btnHeldSource : "-",
        (unsigned long)millis());
}

static int parseOnOff(const char* arg, bool currentVal) {
    if (!arg || !*arg) return !currentVal;
    if (!strcasecmp(arg, "on"))     return 1;
    if (!strcasecmp(arg, "off"))    return 0;
    if (!strcasecmp(arg, "toggle") || !strcasecmp(arg, "t")) return !currentVal;
    return -1;
}

static void handleCommand(const String& line) {
    char buf[SERIAL_BUF_MAX + 1];
    strncpy(buf, line.c_str(), SERIAL_BUF_MAX);
    buf[SERIAL_BUF_MAX] = 0;

    char* rest = NULL;
    char* sp = strpbrk(buf, " \t");
    if (sp) {
        *sp = 0; rest = sp + 1;
        while (*rest == ' ' || *rest == '\t') rest++;
        if (*rest == 0) rest = NULL;
    }
    char* cmd = buf;

    if (!strcasecmp(cmd, "help") || !strcmp(cmd, "?")) { printHelp(); }
    else if (!strcasecmp(cmd, "enow")) {
        char* arg = rest ? strtok(rest, " \t") : NULL;
        int v = parseOnOff(arg, espNowEnabled);
        if (v < 0) { Serial.println(F("usage: enow on|off|toggle")); return; }
        espNowEnabled = v;
        if (!espNowEnabled && !garagePulseStart) setLED(false);
        Serial.printf("[CFG] enow=%s\n", espNowEnabled ? "ON" : "OFF");
    }
    else if (!strcasecmp(cmd, "garage")) { garageStart("manual"); }
    else if (!strcasecmp(cmd, "stats"))  { printStats(); }
    else if (!strcasecmp(cmd, "clear")) {
        rxCount = enowTxCount = enowTxFail = 0;
        garageTriggers = 0;
        Serial.println("[CFG] counters cleared");
    }
    else { Serial.printf("[ERR] unknown: %s  (try: help)\n", cmd); }
}

static void readSerial() {
    while (Serial.available()) {
        char c = (char) Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            String cmd = serialBuf; cmd.trim();
            serialBuf = "";
            if (cmd.length() > 0) handleCommand(cmd);
        } else if (serialBuf.length() < SERIAL_BUF_MAX) {
            serialBuf += c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== Wemos D1 Mini ESP32 - CAN Sniffer + ESP-NOW + Garage ===");

    pinMode(GARAGE_RELAY_PIN, OUTPUT);
    digitalWrite(GARAGE_RELAY_PIN, LOW);
    pinMode(LED_PIN, OUTPUT);
    setLED(false);
    pinMode(BTN_BOOT, INPUT_PULLUP);

    initESPNow();
    initCAN();
    printHelp();
    Serial.printf("[GARAGE] Watching CAN IDs 0x%X (Benz speak) + 0x%X (Ford cruise), hold>=%dms\n",
                  BENZ_SWC_CAN_ID, FORD_CLU11_CAN_ID, GARAGE_HOLD_MS);
    Serial.println("BTN: short tap = ESP-NOW toggle, long press >=2s = manual garage trigger");
}

void loop() {
    readSerial();

    // ---- BOOT button: short = ESP-NOW toggle, long = manual garage trigger ----
    int s = digitalRead(BTN_BOOT);
    uint32_t now = millis();
    if (s == LOW && lastBtnState == HIGH) {
        pressMs = now; handledPress = false;
    }
    if (s == LOW && !handledPress && (now - pressMs) >= 2000) {
        garageStart("BTN long-press");
        handledPress = true;
    }
    if (s == HIGH && lastBtnState == LOW) {
        uint32_t held = now - pressMs;
        if (!handledPress && held >= 50 && held < 2000) {
            espNowEnabled = !espNowEnabled;
            Serial.printf("[BTN] enow=%s\n", espNowEnabled ? "ON" : "OFF");
            if (!espNowEnabled && !garagePulseStart) setLED(false);
        }
        handledPress = true;
    }
    lastBtnState = s;

    // ---- Garage hold timer ----
    if (btnHeld && (now - btnPressStartMs) >= GARAGE_HOLD_MS) {
        garageStart(btnHeldSource);
        btnHeld = false; btnPressStartMs = 0; btnHeldSource = "-";
    }
    garageService();

    // ---- CAN RX -> print, watch garage button, ESP-NOW relay ----
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK && !msg.rtr) {
        rxCount++;
        printFrame(msg.identifier, msg.data_length_code, msg.data, msg.extd);
        watchGarageButton(msg);

        if (espNowEnabled) {
            CanFrame frame;
            frame.id  = msg.identifier;
            frame.dlc = msg.data_length_code;
            memset(frame.data, 0, 8);
            memcpy(frame.data, msg.data,
                   msg.data_length_code > 8 ? 8 : msg.data_length_code);
            esp_err_t r = esp_now_send(broadcast, (uint8_t*)&frame, sizeof(frame));
            if (r == ESP_OK) { enowTxCount++; lastActivityMs = now; }
            else             { enowTxFail++; }
        }
    }

    // ---- LED rapid flash on ESP-NOW activity (suppressed during garage pulse) ----
    if (!garagePulseStart) {
        if (espNowEnabled && (now - lastActivityMs) < 300) {
            if (now - lastLedToggle >= 50) {
                lastLedToggle = now;
                setLED(!ledState);
            }
        } else if (ledState) {
            setLED(false);
        }
    }
}
