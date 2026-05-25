#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include "driver/twai.h"

// --- Heltec WiFi LoRa 32 V3 OLED pins ---
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define VEXT_PIN  36

// --- CAN / TWAI pins (TJA1050) ---
#define CAN_TX    48
#define CAN_RX    47

SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);

static uint32_t txCount = 0;
static uint32_t txErrors = 0;

// Demo CAN IDs to cycle through
static const uint32_t demoIDs[] = {
    0x100, 0x200, 0x300, 0x7DF, 0x7E0, 0x123, 0x456, 0x789
};
static const int demoIDCount = sizeof(demoIDs) / sizeof(demoIDs[0]);
static int currentIDIndex = 0;

void resetOLED() {
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(20);
}

void drawScreen(uint32_t canID, const uint8_t *data, uint8_t len, const char *status) {
    display.clear();

    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "CAN TX Demo");

    display.setFont(ArialMT_Plain_10);

    char buf[48];
    snprintf(buf, sizeof(buf), "ID: 0x%03lX  [%d]", (unsigned long)canID, len);
    display.drawString(0, 20, buf);

    // Show data bytes
    char hexBuf[32] = "";
    int pos = 0;
    for (int i = 0; i < len && pos < (int)sizeof(hexBuf) - 4; i++) {
        pos += snprintf(hexBuf + pos, sizeof(hexBuf) - pos, "%02X ", data[i]);
    }
    display.drawString(0, 32, hexBuf);

    snprintf(buf, sizeof(buf), "Sent: %lu  Err: %lu", (unsigned long)txCount, (unsigned long)txErrors);
    display.drawString(0, 44, buf);

    display.drawString(0, 54, status);

    display.display();
}

void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[CAN] Driver install FAILED: 0x%X\n", err);
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 20, "CAN FAIL!");
        display.display();
        while (true) delay(1000);
    }

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[CAN] Start FAILED: 0x%X\n", err);
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 20, "CAN Start FAIL!");
        display.display();
        while (true) delay(1000);
    }

    Serial.println("[CAN] Started @ 500 kbps");
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== ESP32 LoRa V3 - CAN TX Demo ===");
    Serial.printf("CAN TX=GPIO%d  RX=GPIO%d  TJA1050\n", CAN_TX, CAN_RX);

    // Power on OLED
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(50);

    resetOLED();
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 20, "CAN Init...");
    display.display();

    initCAN();

    uint8_t zeros[8] = {0};
    drawScreen(0x000, zeros, 0, "Ready");
    delay(500);
}

void loop() {
    uint32_t canID = demoIDs[currentIDIndex];
    currentIDIndex = (currentIDIndex + 1) % demoIDCount;

    twai_message_t msg;
    msg.identifier = canID;
    msg.extd = 0;          // Standard 11-bit ID
    msg.rtr = 0;
    msg.data_length_code = 8;

    // Fill with demo data: counter + ID bytes + padding
    msg.data[0] = (uint8_t)(txCount & 0xFF);
    msg.data[1] = (uint8_t)((txCount >> 8) & 0xFF);
    msg.data[2] = (uint8_t)((canID >> 8) & 0xFF);
    msg.data[3] = (uint8_t)(canID & 0xFF);
    msg.data[4] = (uint8_t)(millis() & 0xFF);
    msg.data[5] = (uint8_t)((millis() >> 8) & 0xFF);
    msg.data[6] = 0xCA;
    msg.data[7] = 0xFE;

    Serial.printf("[TX #%lu] ID=0x%03lX Data=", (unsigned long)txCount + 1, (unsigned long)canID);
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", msg.data[i]);

    esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (result == ESP_OK) {
        txCount++;
        Serial.println("OK");
        drawScreen(canID, msg.data, 8, "TX OK");
    } else {
        txErrors++;
        Serial.printf("FAIL (0x%X)\n", result);
        drawScreen(canID, msg.data, 8, "TX FAIL");
    }

    delay(1000);
}
