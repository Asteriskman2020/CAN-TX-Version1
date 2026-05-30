// ============================================================================
//  ESP32-C3 SuperMini - CAN to ESP-NOW Bridge + Active CAN (Serial Console)
//  ----------------------------------------------------------------------------
//  No web portal, no WiFi AP, no DNS, no HTML. Everything is driven over the
//  built-in USB-CDC serial console at 115200 baud.
//
//  Features:
//    - CAN RX via TJA1050 (TX=GP20, RX=GP21) @ 500 kbps, accept-all filter
//    - ESP-NOW broadcast forwarding of every received CAN frame
//    - OBD2 active poller: rotates mode-1 PID requests to 0x7DF, decodes
//      0x7E8..0x7EF replies (RPM, speed, coolant, throttle, etc.)
//    - Manual CAN frame TX over serial (standard / extended / RTR)
//    - BOOT button (GP9): short tap = ESP-NOW relay toggle,
//                         long press >=2s = OBD2 poll toggle
//    - LED (GP8): rapid flash on ESP-NOW relay activity
//
//  Serial commands (numbers in HEX, case-insensitive):
//    help                       show help
//    tx  <id> [b0 b1 ...]       transmit standard CAN frame
//    txe <id> [b0 b1 ...]       transmit extended-ID CAN frame
//    txr <id> [dlc]             transmit RTR (remote) frame
//    poll on|off|toggle         OBD2 active polling
//    enow on|off|toggle         ESP-NOW relay
//    pids                       latest OBD2 decoded values
//    stats                      counters + flags
//    clear                      reset counters
//
//  WARNING: Active CAN can affect vehicle systems. Test on a bench / non-
//  critical bus first.
//
//  Hardware: ESP32-C3 SuperMini + TJA1050 (with 120 ohm termination)
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"

// ---- Pins ----
#define BTN_BOOT  9
#define LED_PIN   8
#define CAN_TX    20
#define CAN_RX    21

// ---- ESP-NOW payload (compatible with v1/v2 receivers) ----
struct __attribute__((packed)) CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

struct ObdPidDef {
    uint8_t     pid;
    const char* name;
    const char* unit;
};

struct ObdValue {
    float    value;
    int      decimals;
    uint32_t timestamp;
    bool     valid;
};

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

static uint32_t rxCount     = 0;
static uint32_t enowTxCount = 0;
static uint32_t enowTxFail  = 0;
static uint32_t canTxCount  = 0;
static uint32_t canTxFail   = 0;
static bool     espNowEnabled  = true;
static bool     obdPollEnabled = false;

static ObdValue obdValues[N_PIDS];
static int      pollCursor = 0;
static uint32_t lastPollMs = 0;
static const uint32_t POLL_INTERVAL_MS = 150;

static int      lastBtnState   = HIGH;
static uint32_t pressMs        = 0;
static bool     handledPress   = false;
static uint32_t lastLedToggle  = 0;
static uint32_t lastActivityMs = 0;
static bool     ledState       = false;

static String  serialBuf;
static const size_t SERIAL_BUF_MAX = 96;

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
    Serial.println("[CAN] Started @ 500 kbps");
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

// Silent variant for the poller so the serial stream stays readable
static void sendObd2Request(uint8_t pid) {
    uint8_t d[8] = {0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendCanFrame(0x7DF, 8, d, false, false);
}

static void decodeObd2Response(const twai_message_t& msg) {
    if (msg.identifier < 0x7E8 || msg.identifier > 0x7EF) return;
    if (msg.data_length_code < 3) return;
    if (msg.data[1] != 0x41) return;
    uint8_t pid = msg.data[2];
    uint8_t A = msg.data_length_code > 3 ? msg.data[3] : 0;
    uint8_t B = msg.data_length_code > 4 ? msg.data[4] : 0;

    float v;
    int decimals = 0;
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
            obdValues[i].value     = v;
            obdValues[i].decimals  = decimals;
            obdValues[i].timestamp = millis();
            obdValues[i].valid     = true;
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
    Serial.println(F("  stats                    counters + flags"));
    Serial.println(F("  clear                    reset counters"));
}

static void printStats() {
    Serial.printf("[STAT] rx=%lu enowtx=%lu enowfail=%lu cantx=%lu ctxf=%lu"
                  " | enow=%s poll=%s up=%lums\n",
        (unsigned long)rxCount, (unsigned long)enowTxCount, (unsigned long)enowTxFail,
        (unsigned long)canTxCount, (unsigned long)canTxFail,
        espNowEnabled  ? "ON" : "OFF",
        obdPollEnabled ? "ON" : "OFF",
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

// -1=invalid, 0=off, 1=on
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
        else    Serial.printf("[TX FAIL] 0x%lX (no ACK / bus error)\n",
                              (unsigned long)id);
        return;
    }
    uint8_t bytes[8] = {0};
    uint8_t n = 0;
    while ((tok = strtok(NULL, " ,\t")) && n < 8) {
        bytes[n++] = (uint8_t)strtoul(tok, NULL, 16);
    }
    bool ok = sendCanFrame(id, n, bytes, extended, false);
    if (ok) printFrame("TX", id, n, bytes, extended);
    else    Serial.printf("[TX FAIL] 0x%lX (no ACK / bus error)\n",
                          (unsigned long)id);
}

static void handleCommand(const String& line) {
    char buf[SERIAL_BUF_MAX + 1];
    strncpy(buf, line.c_str(), SERIAL_BUF_MAX);
    buf[SERIAL_BUF_MAX] = 0;

    // Split into cmd + rest
    char* rest = NULL;
    char* sp = strpbrk(buf, " \t");
    if (sp) {
        *sp = 0;
        rest = sp + 1;
        while (*rest == ' ' || *rest == '\t') rest++;
        if (*rest == 0) rest = NULL;
    }
    char* cmd = buf;

    if (!strcasecmp(cmd, "help") || !strcmp(cmd, "?")) {
        printHelp();
    } else if (!strcasecmp(cmd, "tx")) {
        cmdTx(rest, false, false);
    } else if (!strcasecmp(cmd, "txe")) {
        cmdTx(rest, true,  false);
    } else if (!strcasecmp(cmd, "txr")) {
        cmdTx(rest, false, true);
    } else if (!strcasecmp(cmd, "poll")) {
        char* arg = rest ? strtok(rest, " \t") : NULL;
        int v = parseOnOff(arg, obdPollEnabled);
        if (v < 0) { Serial.println(F("usage: poll on|off|toggle")); return; }
        obdPollEnabled = v;
        Serial.printf("[CFG] poll=%s\n", obdPollEnabled ? "ON" : "OFF");
    } else if (!strcasecmp(cmd, "enow")) {
        char* arg = rest ? strtok(rest, " \t") : NULL;
        int v = parseOnOff(arg, espNowEnabled);
        if (v < 0) { Serial.println(F("usage: enow on|off|toggle")); return; }
        espNowEnabled = v;
        if (!espNowEnabled) setLED(false);
        Serial.printf("[CFG] enow=%s\n", espNowEnabled ? "ON" : "OFF");
    } else if (!strcasecmp(cmd, "pids")) {
        printPids();
    } else if (!strcasecmp(cmd, "stats")) {
        printStats();
    } else if (!strcasecmp(cmd, "clear")) {
        rxCount = enowTxCount = enowTxFail = canTxCount = canTxFail = 0;
        Serial.println("[CFG] counters cleared");
    } else {
        Serial.printf("[ERR] unknown: %s  (try: help)\n", cmd);
    }
}

static void readSerial() {
    while (Serial.available()) {
        char c = (char) Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            String cmd = serialBuf;
            cmd.trim();
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
    Serial.println("\n=== SuperMini CAN-ESP-NOW Bridge (Serial Console) ===");

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
    Serial.println("BTN: short tap = ESP-NOW toggle, long press >=2s = OBD2 poll toggle");
}

void loop() {
    readSerial();

    // Button
    int s = digitalRead(BTN_BOOT);
    uint32_t now = millis();
    if (s == LOW && lastBtnState == HIGH) {
        pressMs = now;
        handledPress = false;
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
            if (!espNowEnabled) setLED(false);
        }
        handledPress = true;
    }
    lastBtnState = s;

    // OBD2 poller
    if (obdPollEnabled && (now - lastPollMs) >= POLL_INTERVAL_MS) {
        lastPollMs = now;
        sendObd2Request(OBD_PIDS[pollCursor].pid);
        pollCursor = (pollCursor + 1) % N_PIDS;
    }

    // CAN RX -> print, decode, optionally relay
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK && !msg.rtr) {
        rxCount++;
        printFrame("RX", msg.identifier, msg.data_length_code, msg.data, msg.extd);
        decodeObd2Response(msg);

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

    // LED rapid flash on relay activity
    if (espNowEnabled && (now - lastActivityMs) < 300) {
        if (now - lastLedToggle >= 50) {
            lastLedToggle = now;
            setLED(!ledState);
        }
    } else if (ledState) {
        setLED(false);
    }
}
