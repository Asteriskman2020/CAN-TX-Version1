// ============================================================================
//  Wemos D1 Mini ESP32 — CAN ↔ ESP-NOW Bridge + Active CAN + Garage Opener
//  ----------------------------------------------------------------------------
//  Merged sketch:
//    - v3 CAN sniffer/bridge (from Asteriskman2020/CAN-TX-Version1):
//        * CAN RX via TJA1050 @ 500 kbps, ESP-NOW broadcast forwarding
//        * Active CAN: OBD2 mode-1 poller + manual TX from serial REPL
//        * BOOT button: short tap = ESP-NOW toggle, long press = poll toggle
//    - Garage opener (from idealabkmutt/ODB2/FORD_Benz_garage_opener_2PID):
//        * Benz S320 0x3F6 bit 12  = "Speak" button on steering wheel
//        * Ford Ranger 0x09A bit 56 = Cruise main switch
//        * Either button held >= 3 s -> fires local relay for 1 s
//
//  Pins (RELAY + LED taken from garage opener; CAN pins added):
//        GARAGE_RELAY = GPIO5   (D1 silkscreen on D1-mini boards)
//        LED          = GPIO2   (D4, active-LOW built-in LED)
//        CAN_TX       = GPIO21  (to TJA1050 CTX)
//        CAN_RX       = GPIO22  (to TJA1050 CRX)
//        BTN_BOOT     = GPIO0   (on-board BOOT button)
//
//  Serial REPL (115200, hex unless noted):
//        help / tx / txe / txr / poll / enow / pids / stats / clear
//        garage           -- manually pulse the relay (test)
//
//  WARNING: Active CAN can affect vehicle systems. Bench-test first.
//  Hardware: Wemos D1 Mini ESP32 (WROOM-32) + TJA1050 (120Ω termination)
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"

// ---- Pins ----
#define GARAGE_RELAY_PIN  5    // from FORD_Benz_garage_opener
#define LED_PIN           2    // from FORD_Benz_garage_opener (active-LOW)
#define CAN_TX           21
#define CAN_RX           22
#define BTN_BOOT          0    // BOOT button on classic ESP32 dev boards

// ---- Garage opener tuning ----
#define BENZ_SWC_CAN_ID         0x3F6   // 1014 - VCLEFT_switchStatus (Speak btn bit 12)
#define FORD_CLU11_CAN_ID       0x09A   // 154  - CLU11 (Cruise btn bit 56)
#define GARAGE_HOLD_MS          3000    // button must be held this long
#define GARAGE_PULSE_MS         1000    // relay HIGH duration
#define GARAGE_REARM_MS         2000    // cooldown before next trigger

// ---- ESP-NOW payload (compatible with v1/v2/v3 receivers on this repo) ----
struct __attribute__((packed)) CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

// ---- OBD2 PID table ----
struct ObdPidDef { uint8_t pid; const char* name; const char* unit; };
struct ObdValue  { float value; int decimals; uint32_t timestamp; bool valid; };

static const ObdPidDef OBD_PIDS[] = {
    {0x04, "load",     "%"},
    {0x05, "coolant",  "C"},
    {0x0C, "rpm",      ""},
    {0x0D, "speed",    "km/h"},
    {0x0F, "iat",      "C"},
    {0x11, "throttle", "%"},
    {0x2F, "fuel",     "%"},
    {0x42, "voltage",  "V"},
};
static const int N_PIDS = sizeof(OBD_PIDS) / sizeof(OBD_PIDS[0]);

// ---- Globals ----
static uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint32_t rxCount = 0, enowTxCount = 0, enowTxFail = 0;
static uint32_t canTxCount = 0, canTxFail = 0;
static uint32_t garageTriggers = 0;
static bool espNowEnabled  = true;
static bool obdPollEnabled = false;

static ObdValue obdValues[N_PIDS];
static int      pollCursor = 0;
static uint32_t lastPollMs = 0;
static const uint32_t POLL_INTERVAL_MS = 150;

// Garage state machine
static bool     btnHeld          = false;
static uint32_t btnPressStartMs  = 0;
static const char* btnHeldSource = "-";
static uint32_t garagePulseStart = 0;
static uint32_t lastTriggerMs    = 0;

// Button toggles for ESP-NOW / poll
static int      lastBtnState   = HIGH;
static uint32_t pressMs        = 0;
static bool     handledPress   = false;
static uint32_t lastLedToggle  = 0;
static uint32_t lastActivityMs = 0;
static bool     ledState       = false;

static String  serialBuf;
static const size_t SERIAL_BUF_MAX = 96;

// ---- LED (active-LOW) ----
static void setLED(bool on) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);
    ledState = on;
}

// ---- Print helpers ----
static void printFrame(const char* tag, uint32_t id, uint8_t dlc,
                       const uint8_t* data, bool ext) {
    Serial.printf("[%s] 0x%lX%s dlc=%u", tag, (unsigned long)id,
                  ext ? "X" : "", dlc);
    for (int i = 0; i < dlc && i < 8; i++) Serial.printf(" %02X", data[i]);
    Serial.println();
}

// ---- CAN ----
static void initCAN() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
    Serial.println("[CAN] Started @ 500 kbps (TX=GPIO21 RX=GPIO22)");
}

static bool sendCanFrame(uint32_t id, uint8_t dlc, const uint8_t* data,
                         bool extended, bool rtr) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = dlc > 8 ? 8 : dlc;
    msg.extd = extended ? 1 : 0;
    msg.rtr  = rtr ? 1 : 0;
    if (!rtr && data) memcpy(msg.data, data, msg.data_length_code);
    esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(20));
    if (r == ESP_OK) { canTxCount++; return true; }
    canTxFail++;
    return false;
}

static void sendObd2Request(uint8_t pid) {
    uint8_t d[8] = {0x02, 0x01, pid, 0, 0, 0, 0, 0};
    sendCanFrame(0x7DF, 8, d, false, false);
}

static void decodeObd2Response(const twai_message_t& msg) {
    if (msg.identifier < 0x7E8 || msg.identifier > 0x7EF) return;
    if (msg.data_length_code < 3 || msg.data[1] != 0x41) return;
    uint8_t pid = msg.data[2];
    uint8_t A = msg.data_length_code > 3 ? msg.data[3] : 0;
    uint8_t B = msg.data_length_code > 4 ? msg.data[4] : 0;

    float v; int decimals = 0;
    switch (pid) {
        case 0x04: v = A * 100.0f / 255.0f;      decimals = 1; break;
        case 0x05: v = (float)(A - 40);          break;
        case 0x0C: v = ((A * 256.0f) + B) / 4.0f; break;
        case 0x0D: v = (float)A;                 break;
        case 0x0F: v = (float)(A - 40);          break;
        case 0x11: v = A * 100.0f / 255.0f;      decimals = 1; break;
        case 0x2F: v = A * 100.0f / 255.0f;      decimals = 1; break;
        case 0x42: v = ((A * 256.0f) + B) / 1000.0f; decimals = 2; break;
        default: return;
    }
    for (int i = 0; i < N_PIDS; i++) {
        if (OBD_PIDS[i].pid == pid) {
            obdValues[i].value = v;
            obdValues[i].decimals = decimals;
            obdValues[i].timestamp = millis();
            obdValues[i].valid = true;
            Serial.printf("[OBD] %s=%.*f %s\n",
                          OBD_PIDS[i].name, decimals, v, OBD_PIDS[i].unit);
            return;
        }
    }
}

// ---- ESP-NOW (no AP, no STA association) ----
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
// Bit numbering matches the DBC convention used in the source sketch:
// raw_ul = d[0..7] interpreted little-endian -> bit N = byte (N/8) bit (N%8).
//   Benz speak (bit 12, len 1) -> data[1] >> 4 & 1
//   Ford cruise (bit 56, len 1) -> data[7] & 1
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
    if (garagePulseStart != 0) return;                       // already running
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
    Serial.println(F("Commands (hex unless noted):"));
    Serial.println(F("  help                     this help"));
    Serial.println(F("  tx  <id> [b0 b1 ...]     send standard CAN frame"));
    Serial.println(F("  txe <id> [b0 b1 ...]     send extended-ID frame"));
    Serial.println(F("  txr <id> [dlc]           send RTR frame"));
    Serial.println(F("  poll on|off|toggle       OBD2 active poller"));
    Serial.println(F("  enow on|off|toggle       ESP-NOW relay"));
    Serial.println(F("  pids                     latest decoded OBD2 values"));
    Serial.println(F("  garage                   manually pulse the relay"));
    Serial.println(F("  stats                    counters + flags"));
    Serial.println(F("  clear                    reset counters"));
}

static void printStats() {
    Serial.printf("[STAT] rx=%lu enowtx=%lu enowfail=%lu cantx=%lu ctxf=%lu gates=%lu"
                  " | enow=%s poll=%s held=%s up=%lums\n",
        (unsigned long)rxCount, (unsigned long)enowTxCount, (unsigned long)enowTxFail,
        (unsigned long)canTxCount, (unsigned long)canTxFail,
        (unsigned long)garageTriggers,
        espNowEnabled  ? "ON" : "OFF",
        obdPollEnabled ? "ON" : "OFF",
        btnHeld ? btnHeldSource : "-",
        (unsigned long)millis());
}

static void printPids() {
    uint32_t now = millis();
    Serial.println(F("[PIDS]"));
    for (int i = 0; i < N_PIDS; i++) {
        if (obdValues[i].valid) {
            Serial.printf("  %-10s %.*f %-5s  (%lums ago)\n",
                OBD_PIDS[i].name, obdValues[i].decimals, obdValues[i].value,
                OBD_PIDS[i].unit, (unsigned long)(now - obdValues[i].timestamp));
        } else {
            Serial.printf("  %-10s --     %s\n", OBD_PIDS[i].name, OBD_PIDS[i].unit);
        }
    }
}

static int parseOnOff(const char* arg, bool currentVal) {
    if (!arg || !*arg) return !currentVal;
    if (!strcasecmp(arg, "on"))     return 1;
    if (!strcasecmp(arg, "off"))    return 0;
    if (!strcasecmp(arg, "toggle") || !strcasecmp(arg, "t")) return !currentVal;
    return -1;
}

static void cmdTx(char* args, bool extended, bool rtr) {
    if (!args) { Serial.println(F("usage: tx <id> [b0 b1 ...]")); return; }
    char* tok = strtok(args, " ,\t");
    if (!tok) { Serial.println(F("usage: tx <id> [b0 b1 ...]")); return; }
    uint32_t id = strtoul(tok, NULL, 16);
    if (rtr) {
        tok = strtok(NULL, " ,\t");
        uint8_t dlc = tok ? (uint8_t)strtoul(tok, NULL, 16) : 0;
        if (dlc > 8) dlc = 8;
        bool ok = sendCanFrame(id, dlc, NULL, extended, true);
        if (ok) Serial.printf("[TX RTR] 0x%lX%s dlc=%u\n",
                              (unsigned long)id, extended ? "X" : "", dlc);
        else    Serial.printf("[TX FAIL] 0x%lX (no ACK / bus error)\n", (unsigned long)id);
        return;
    }
    uint8_t bytes[8] = {0}; uint8_t n = 0;
    while ((tok = strtok(NULL, " ,\t")) && n < 8) {
        bytes[n++] = (uint8_t)strtoul(tok, NULL, 16);
    }
    bool ok = sendCanFrame(id, n, bytes, extended, false);
    if (ok) printFrame("TX", id, n, bytes, extended);
    else    Serial.printf("[TX FAIL] 0x%lX (no ACK / bus error)\n", (unsigned long)id);
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

    if (!strcasecmp(cmd, "help") || !strcmp(cmd, "?"))      { printHelp(); }
    else if (!strcasecmp(cmd, "tx"))    { cmdTx(rest, false, false); }
    else if (!strcasecmp(cmd, "txe"))   { cmdTx(rest, true,  false); }
    else if (!strcasecmp(cmd, "txr"))   { cmdTx(rest, false, true);  }
    else if (!strcasecmp(cmd, "poll")) {
        char* arg = rest ? strtok(rest, " \t") : NULL;
        int v = parseOnOff(arg, obdPollEnabled);
        if (v < 0) { Serial.println(F("usage: poll on|off|toggle")); return; }
        obdPollEnabled = v;
        Serial.printf("[CFG] poll=%s\n", obdPollEnabled ? "ON" : "OFF");
    }
    else if (!strcasecmp(cmd, "enow")) {
        char* arg = rest ? strtok(rest, " \t") : NULL;
        int v = parseOnOff(arg, espNowEnabled);
        if (v < 0) { Serial.println(F("usage: enow on|off|toggle")); return; }
        espNowEnabled = v;
        if (!espNowEnabled && !garagePulseStart) setLED(false);
        Serial.printf("[CFG] enow=%s\n", espNowEnabled ? "ON" : "OFF");
    }
    else if (!strcasecmp(cmd, "pids"))  { printPids(); }
    else if (!strcasecmp(cmd, "stats")) { printStats(); }
    else if (!strcasecmp(cmd, "clear")) {
        rxCount = enowTxCount = enowTxFail = canTxCount = canTxFail = 0;
        garageTriggers = 0;
        Serial.println("[CFG] counters cleared");
    }
    else if (!strcasecmp(cmd, "garage")) { garageStart("manual"); }
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
    Serial.println("\n=== Wemos D1 Mini ESP32 - CAN/ESP-NOW Bridge + Garage Opener ===");

    pinMode(GARAGE_RELAY_PIN, OUTPUT);
    digitalWrite(GARAGE_RELAY_PIN, LOW);
    pinMode(LED_PIN, OUTPUT);
    setLED(false);
    pinMode(BTN_BOOT, INPUT_PULLUP);

    for (int i = 0; i < N_PIDS; i++) {
        obdValues[i].valid = false;
        obdValues[i].value = 0;
        obdValues[i].decimals = 0;
        obdValues[i].timestamp = 0;
    }

    initESPNow();
    initCAN();
    printHelp();
    Serial.printf("[GARAGE] Watching CAN IDs 0x%X (Benz speak) + 0x%X (Ford cruise), hold>=%dms\n",
                  BENZ_SWC_CAN_ID, FORD_CLU11_CAN_ID, GARAGE_HOLD_MS);
    Serial.println("BTN: short tap = ESP-NOW toggle, long press >=2s = OBD2 poll toggle");
}

void loop() {
    readSerial();

    // ---- BOOT button: short = ESP-NOW toggle, long = OBD2 poll toggle ----
    int s = digitalRead(BTN_BOOT);
    uint32_t now = millis();
    if (s == LOW && lastBtnState == HIGH) {
        pressMs = now; handledPress = false;
    }
    if (s == LOW && !handledPress && (now - pressMs) >= 2000) {
        obdPollEnabled = !obdPollEnabled;
        Serial.printf("[BTN] poll=%s\n", obdPollEnabled ? "ON" : "OFF");
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

    // ---- OBD2 poller ----
    if (obdPollEnabled && (now - lastPollMs) >= POLL_INTERVAL_MS) {
        lastPollMs = now;
        sendObd2Request(OBD_PIDS[pollCursor].pid);
        pollCursor = (pollCursor + 1) % N_PIDS;
    }

    // ---- Garage hold timer ----
    if (btnHeld && (now - btnPressStartMs) >= GARAGE_HOLD_MS) {
        garageStart(btnHeldSource);
        btnHeld = false; btnPressStartMs = 0; btnHeldSource = "-";
    }
    garageService();

    // ---- CAN RX -> print, decode OBD, watch garage button, ESP-NOW relay ----
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK && !msg.rtr) {
        rxCount++;
        printFrame("RX", msg.identifier, msg.data_length_code, msg.data, msg.extd);
        decodeObd2Response(msg);
        watchGarageButton(msg);

        if (espNowEnabled) {
            CanFrame frame;
            frame.id = msg.identifier;
            frame.dlc = msg.data_length_code;
            memset(frame.data, 0, 8);
            memcpy(frame.data, msg.data,
                   msg.data_length_code > 8 ? 8 : msg.data_length_code);
            esp_err_t r = esp_now_send(broadcast, (uint8_t*)&frame, sizeof(frame));
            if (r == ESP_OK) { enowTxCount++; lastActivityMs = now; }
            else             { enowTxFail++; }
        }
    }

    // ---- LED rapid-flash on relay activity (suppressed during garage pulse) ----
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
