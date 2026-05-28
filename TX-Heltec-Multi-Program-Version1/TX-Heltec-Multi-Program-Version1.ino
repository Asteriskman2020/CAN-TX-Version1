// ============================================================================
//  Heltec WiFi LoRa 32 V3 - Multi-Program OBD CAN+ESP-NOW TX (v1)
//  ----------------------------------------------------------------------------
//  Loops through 5 captured Benz S320 OBD-II programs, sending only
//  Speed (PID 0x0D), RPM (PID 0x0C), and Coolant Temp (PID 0x05) frames
//  via both the CAN bus (TJA1050) and ESP-NOW broadcast.
//
//  Hardware:
//    - Heltec WiFi LoRa 32 V3 (ESP32-S3 + SSD1306 OLED + LoRa)
//    - TJA1050 CAN transceiver wired to GPIO48 (TX) / GPIO47 (RX)
//    - PRG button on GPIO0 (active LOW)
//
//  Controls (PRG button):
//    - Double tap  -> cycle playback speed (1x / 2x / 5x / 10x)
//                     2x = skip 2 frames per tick, etc.
//    - Hold 3 sec  -> switch to next program (P1 .. P5)
//
//  Programs (compiled-in PROGMEM tables, see benz_data.h):
//    P1 Cold Start   (7566 frames)  - Benz cold start.csv
//    P2 B3 (warm)    (2500 frames)  - b3.csv
//    P3 B2           ( 942 frames)  - B2.csv
//    P4 Drive1       ( 532 frames)  - Drive1.csv
//    P5 D2           ( 281 frames)  - d2.csv
//
//  CAN bus: 500 kbps standard ID.  ESP-NOW: broadcast packet of
//    struct CanFrame { uint32_t id; uint8_t dlc; uint8_t data[8]; }
//
//  Required Arduino libraries:
//    - ThingPulse SSD1306 driver
//    - WiFi, esp_now (ESP32 core)
//    - benz_data.h (included alongside this .ino)
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"
#include "benz_data.h"

// --- Heltec WiFi LoRa 32 V3 OLED pins ---
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define VEXT_PIN  36

// --- CAN / TWAI pins (TJA1050) ---
#define CAN_TX    48
#define CAN_RX    47

// --- PRG button (GPIO0, active LOW) ---
#define BTN_PRG   0

SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);

struct __attribute__((packed)) CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

static uint32_t txCount = 0;
static uint32_t txErrors = 0;
static uint32_t espNowOK = 0;
static uint32_t espNowFail = 0;
static int currentFrame = 0;
static bool espNowEnabled = true;
static bool canEnabled = true;

static int currentProgram = 0;  // index into benzPrograms[]

// Playback speed
static const int speedSteps[] = {1, 2, 5, 10};
static const int NUM_SPEEDS = sizeof(speedSteps) / sizeof(speedSteps[0]);
static int speedIdx = 0;

static uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void resetOLED() {
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(20);
}

void drawScreen(uint32_t canID, const uint8_t *data, uint8_t len, const char *status) {
    display.clear();
    display.setColor(WHITE);

    // CAN OFF: full-screen blue alert
    if (!canEnabled) {
        display.fillRect(0, 0, 128, 64);
        display.setColor(BLACK);
        display.setFont(ArialMT_Plain_24);
        int w = display.getStringWidth("CAN OFF");
        display.drawString((128 - w) / 2, 10, "CAN OFF");
        display.setFont(ArialMT_Plain_10);
        display.drawString(8, 44, "Hold PRG 2s to enable");
        display.setColor(WHITE);
        display.display();
        return;
    }

    // Title: program name
    display.setFont(ArialMT_Plain_16);
    char title[24];
    snprintf(title, sizeof(title), "P%d:%s", currentProgram + 1, benzPrograms[currentProgram].name);
    display.drawString(0, 0, title);

    display.setFont(ArialMT_Plain_10);

    // Status row: playback speed + ESP-NOW
    char statBuf[24];
    snprintf(statBuf, sizeof(statBuf), "%dx  NOW:%s",
             speedSteps[speedIdx],
             espNowEnabled ? "ON" : "OFF");
    display.drawString(0, 17, statBuf);

    // Frame ID
    char buf[48];
    snprintf(buf, sizeof(buf), "ID:0x%03lX [%d]", (unsigned long)canID, len);
    display.drawString(0, 28, buf);

    // Data bytes
    char hexBuf[32] = "";
    int pos = 0;
    for (int i = 0; i < len && pos < (int)sizeof(hexBuf) - 4; i++) {
        pos += snprintf(hexBuf + pos, sizeof(hexBuf) - pos, "%02X ", data[i]);
    }
    display.drawString(0, 39, hexBuf);

    // Counter + status
    snprintf(buf, sizeof(buf), "%d/%d %s",
             currentFrame + 1, benzPrograms[currentProgram].count, status);
    display.drawString(0, 52, buf);

    display.display();
}

bool initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] Driver install FAILED!");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] Start FAILED!");
        return false;
    }
    Serial.println("[CAN] Started @ 500 kbps");
    return true;
}

bool initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init FAILED!");
        return false;
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.printf("[ESP-NOW] Ready - MAC: %s\n", WiFi.macAddress().c_str());
    return true;
}

// =====================================================================
// Button handling:
//   - Double tap  -> cycle playback speed (1x / 2x / 5x / 10x)
//   - Hold 3 sec  -> switch to next program
// =====================================================================
static int lastBtnState = HIGH;
static uint32_t pressStartMs = 0;
static uint32_t releaseMs = 0;
static bool longPressHandled = false;
static int tapCount = 0;
static const uint32_t DOUBLE_TAP_WINDOW = 400;  // ms between taps for double-tap
static const uint32_t HOLD_PROG_MS = 3000;      // 3s hold = switch program

void switchProgram() {
    currentProgram = (currentProgram + 1) % BENZ_PROGRAM_COUNT;
    currentFrame = 0;
    Serial.printf("[BTN HOLD] Program %d: %s (%d frames)\n",
                  currentProgram + 1,
                  benzPrograms[currentProgram].name,
                  benzPrograms[currentProgram].count);
}

void cycleSpeed() {
    speedIdx = (speedIdx + 1) % NUM_SPEEDS;
    Serial.printf("[BTN DBLTAP] Playback speed: %dx\n", speedSteps[speedIdx]);
}

void checkButton() {
    int s = digitalRead(BTN_PRG);
    uint32_t now = millis();

    if (s == LOW && lastBtnState == HIGH) {
        // Just pressed
        pressStartMs = now;
        longPressHandled = false;
        Serial.println("[BTN] press");
    }

    if (s == LOW && !longPressHandled && (now - pressStartMs) >= HOLD_PROG_MS) {
        // 3s hold reached - switch program
        switchProgram();
        longPressHandled = true;
        tapCount = 0;
    }

    if (s == HIGH && lastBtnState == LOW) {
        // Just released
        uint32_t held = now - pressStartMs;
        Serial.printf("[BTN] release after %lums\n", (unsigned long)held);
        if (!longPressHandled && held >= 30 && held < HOLD_PROG_MS) {
            tapCount++;
            releaseMs = now;
        }
    }

    // Confirm tap action after window expires (no further press came)
    if (tapCount > 0 && (now - releaseMs) > DOUBLE_TAP_WINDOW) {
        if (tapCount >= 2) {
            cycleSpeed();
        }
        tapCount = 0;
    }

    lastBtnState = s;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Benz Multi-Program TX (CAN + ESP-NOW) ===");
    Serial.printf("Programs available: %d\n", BENZ_PROGRAM_COUNT);
    for (int i = 0; i < BENZ_PROGRAM_COUNT; i++) {
        Serial.printf("  P%d: %s (%d frames)\n", i + 1, benzPrograms[i].name, benzPrograms[i].count);
    }

    pinMode(BTN_PRG, INPUT_PULLUP);

    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(50);

    resetOLED();
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setColor(WHITE);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Multi-Program TX");
    display.drawString(0, 14, "Init CAN...");
    display.display();

    bool canOK = initCAN();
    canEnabled = canOK;

    display.drawString(0, 26, canOK ? "CAN: OK" : "CAN: FAIL (skip)");
    display.drawString(0, 38, "Init ESP-NOW...");
    display.display();

    bool nowOK = initESPNow();
    espNowEnabled = nowOK;

    display.drawString(0, 50, nowOK ? "NOW: OK" : "NOW: FAIL (skip)");
    display.display();
    delay(800);

    uint8_t zeros[8] = {0};
    drawScreen(0x000, zeros, 0, "Ready");
}

static uint32_t lastFrameMs = 0;
static const uint32_t FRAME_INTERVAL_MS = 100;

void sendOneFrame() {
    const BenzProgram &prog = benzPrograms[currentProgram];
    BenzFrame bf;
    memcpy_P(&bf, &prog.frames[currentFrame], sizeof(BenzFrame));

    twai_message_t msg;
    msg.identifier = bf.id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = bf.dlc;
    memcpy(msg.data, bf.data, 8);

    bool canOK = false, canSent = false;
    if (canEnabled) {
        canOK = (twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK);  // shorter timeout
        canSent = true;
    }

    bool nowOK = false, nowSent = false;
    if (espNowEnabled) {
        CanFrame frame;
        frame.id = bf.id;
        frame.dlc = bf.dlc;
        memcpy(frame.data, bf.data, 8);
        nowOK = (esp_now_send(broadcast, (uint8_t *)&frame, sizeof(frame)) == ESP_OK);
        nowSent = true;
    }

    txCount++;
    if (canSent && !canOK) txErrors++;
    if (nowSent) { if (nowOK) espNowOK++; else espNowFail++; }

    Serial.printf("[P%d %d/%d %dx] ID=0x%03lX  CAN:%s NOW:%s\n",
                  currentProgram + 1, currentFrame + 1, prog.count,
                  speedSteps[speedIdx],
                  (unsigned long)bf.id,
                  canSent ? (canOK ? "OK" : "FAIL") : "OFF",
                  nowSent ? (nowOK ? "OK" : "FAIL") : "OFF");

    const char *status;
    if (!canEnabled && !espNowEnabled) status = "BOTH OFF";
    else if (!canEnabled) status = nowOK ? "NOW OK" : "NOW FAIL";
    else if (!espNowEnabled) status = canOK ? "CAN OK" : "CAN FAIL";
    else status = (canOK && nowOK) ? "CAN+NOW" :
                  canOK ? "CAN only" :
                  nowOK ? "NOW only" : "FAIL";

    drawScreen(bf.id, bf.data, bf.dlc, status);

    currentFrame = (currentFrame + speedSteps[speedIdx]) % prog.count;
}

void loop() {
    // Poll button EVERY loop iteration (high responsiveness)
    checkButton();

    uint32_t now = millis();
    if (now - lastFrameMs >= FRAME_INTERVAL_MS) {
        lastFrameMs = now;
        sendOneFrame();
    }

    delay(2);  // ~500Hz button polling
}

