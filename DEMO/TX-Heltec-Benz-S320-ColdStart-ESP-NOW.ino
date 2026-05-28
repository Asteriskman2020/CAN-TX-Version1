#include <Wire.h>
#include <SSD1306Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"

#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define VEXT_PIN  36
#define CAN_TX    48
#define CAN_RX    47

SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);

struct BenzFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

static const BenzFrame PROGMEM benzFrames[] = {
    {0x5, 8, {0x00,0x10,0x00,0x00,0x11,0x00,0x40,0x9A}},
    {0x105, 8, {0x02,0xFC,0x81,0x00,0x00,0x8C,0x93,0x48}},
    {0x203, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x3, 8, {0x10,0x14,0x10,0x00,0x04,0xFF,0xB0,0x67}},
    {0x7DF, 8, {0x02,0x01,0x10,0x00,0x00,0x00,0x00,0x00}},
    {0x7E8, 8, {0x03,0x41,0x10,0x58,0x00,0x00,0x00,0x00}},
    {0x75E, 8, {0x06,0x02,0x00,0x10,0x70,0x08,0x86,0x00}},
    {0x201, 8, {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x245, 8, {0x7F,0xBC,0x80,0x97,0x85,0x81,0x40,0x78}},
    {0x766, 8, {0x02,0xA3,0x02,0xC5,0x02,0x4B,0x00,0x00}},
    {0x205, 8, {0x82,0x8C,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x10C, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x375, 8, {0x00,0x57,0x35,0x35,0x3F,0x07,0x00,0x00}},
    {0x12D, 8, {0x0F,0x0F,0x00,0x00,0x10,0xB3,0x07,0x21}},
    {0x1, 8, {0x4C,0xC0,0x9F,0x94,0x59,0x00,0x37,0x92}},
    {0x3F6, 5, {0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x7, 6, {0x00,0x85,0x00,0x00,0x3A,0xD4,0x00,0x00}},
    {0x39D, 8, {0x89,0x7F,0x00,0x52,0x00,0x08,0x00,0x4F}},
    {0x76E, 8, {0x01,0x05,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x776, 8, {0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x7E9, 8, {0x04,0x41,0x0C,0x0B,0xE4,0xFF,0xFF,0xFF}},
    {0x39F, 8, {0x14,0x37,0x1B,0x10,0x01,0x19,0x02,0xFF}},
    {0x19F, 8, {0x00,0x00,0x8F,0x15,0x9A,0x16,0xA5,0xE4}},
    {0x400, 8, {0xFC,0x00,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF}},
    {0x207, 8, {0x00,0x00,0x1D,0x41,0x83,0xDB,0x3F,0xE1}},
    {0x206, 8, {0x00,0x00,0xBE,0xA2,0x00,0x25,0xFA,0xF1}},
    {0x3F4, 8, {0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00}},
    {0x3E3, 8, {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
    {0x3EB, 8, {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
    {0x3E9, 8, {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
    {0x3E7, 8, {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
    {0x3E5, 8, {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
    {0x1C1, 4, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},};

static const int BENZ_FRAME_COUNT = sizeof(benzFrames) / sizeof(benzFrames[0]);

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
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Benz ColdStart");
    display.setFont(ArialMT_Plain_10);
    char buf[48];
    snprintf(buf, sizeof(buf), "ID: 0x%03lX  [%d]", (unsigned long)canID, len);
    display.drawString(0, 18, buf);
    char hexBuf[32] = "";
    int pos = 0;
    for (int i = 0; i < len && pos < (int)sizeof(hexBuf) - 4; i++) {
        pos += snprintf(hexBuf + pos, sizeof(hexBuf) - pos, "%02X ", data[i]);
    }
    display.drawString(0, 30, hexBuf);
    snprintf(buf, sizeof(buf), "%d/%d CAN:%lu", currentFrame + 1, BENZ_FRAME_COUNT, (unsigned long)txCount);
    display.drawString(0, 42, buf);
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

void initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init FAILED!");
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 20, "NOW FAIL!");
        display.display();
        while (true) delay(1000);
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.printf("[ESP-NOW] Ready - MAC: %s\n", WiFi.macAddress().c_str());
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Benz S320 Cold Start - CAN + ESP-NOW TX ===");
    Serial.printf("CAN TX=GPIO%d  RX=GPIO%d\n", CAN_TX, CAN_RX);
    Serial.printf("Benz frames: %d (cold start unique IDs)\n", BENZ_FRAME_COUNT);
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(50);
    resetOLED();
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 20, "Init...");
    display.display();
    initCAN();
    initESPNow();
    uint8_t zeros[8] = {0};
    drawScreen(0x000, zeros, 0, "Ready");
    delay(500);
}

void loop() {
    BenzFrame bf;
    memcpy_P(&bf, &benzFrames[currentFrame], sizeof(BenzFrame));
    twai_message_t msg;
    msg.identifier = bf.id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = bf.dlc;
    memcpy(msg.data, bf.data, 8);
    esp_err_t canResult = twai_transmit(&msg, pdMS_TO_TICKS(100));
    bool canOK = (canResult == ESP_OK);
    CanFrame frame;
    frame.id = bf.id;
    frame.dlc = bf.dlc;
    memcpy(frame.data, bf.data, 8);
    esp_err_t nowResult = esp_now_send(broadcast, (uint8_t *)&frame, sizeof(frame));
    bool nowOK = (nowResult == ESP_OK);
    txCount++;
    if (!canOK) txErrors++;
    if (nowOK) espNowOK++; else espNowFail++;
    Serial.printf("[TX %d/%d] ID=0x%03lX [%d] ",
                  currentFrame + 1, BENZ_FRAME_COUNT,
                  (unsigned long)bf.id, bf.dlc);
    for (int i = 0; i < bf.dlc; i++) Serial.printf("%02X ", bf.data[i]);
    Serial.printf("CAN:%s NOW:%s\n", canOK ? "OK" : "FAIL", nowOK ? "OK" : "FAIL");
    const char *status = (canOK && nowOK) ? "CAN+NOW OK" :
                         canOK ? "CAN OK / NOW FAIL" :
                         nowOK ? "CAN FAIL / NOW OK" : "BOTH FAIL";
    drawScreen(bf.id, bf.data, bf.dlc, status);
    currentFrame = (currentFrame + 1) % BENZ_FRAME_COUNT;
    delay(1000);
}
