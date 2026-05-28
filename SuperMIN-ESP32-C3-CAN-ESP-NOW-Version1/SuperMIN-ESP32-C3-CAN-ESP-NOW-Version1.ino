// ============================================================================
//  ESP32-C3 SuperMini - CAN to ESP-NOW Bridge
//  ----------------------------------------------------------------------------
//  Reads frames from the CAN bus (TJA1050 transceiver) and forwards each
//  valid frame as an ESP-NOW broadcast packet. The built-in LED flashes very
//  quickly while CAN frames are being relayed; the BOOT button toggles the
//  ESP-NOW relay on/off.
//
//  Hardware:
//    - ESP32-C3 SuperMini
//    - TJA1050 CAN transceiver
//        TXD -> GP20  (CAN_TX)
//        RXD -> GP21  (CAN_RX)
//    - Built-in BOOT button -> GP9 (active LOW)
//    - Built-in LED -> GP8 (active LOW)
//
//  CAN bus: 500 kbps, standard 11-bit IDs accepted
//  ESP-NOW: broadcast (FF:FF:FF:FF:FF:FF), no peer pairing needed
// ============================================================================

#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"

#define BTN_BOOT  9
#define LED_PIN   8
#define CAN_TX    20
#define CAN_RX    21

struct __attribute__((packed)) CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

static uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint32_t rxCount = 0;
static uint32_t txCount = 0;
static uint32_t txFail = 0;
static volatile bool espNowEnabled = true;

static int lastBtnState = HIGH;
static uint32_t pressMs = 0;
static bool handledPress = false;

static uint32_t lastLedToggle = 0;
static uint32_t lastActivityMs = 0;
static bool ledState = false;
static const uint32_t LED_FLASH_INTERVAL = 50;
static const uint32_t ACTIVITY_TIMEOUT = 300;

void setLED(bool on) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);
    ledState = on;
}

void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] Driver install FAILED!");
        while (true) { setLED(true); delay(200); setLED(false); delay(200); }
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] Start FAILED!");
        while (true) { setLED(true); delay(200); setLED(false); delay(200); }
    }
    Serial.printf("[CAN] Started @ 500 kbps  TX=GPIO%d RX=GPIO%d\n", CAN_TX, CAN_RX);
}

void initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init FAILED!");
        while (true) { setLED(true); delay(500); setLED(false); delay(500); }
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.printf("[ESP-NOW] Ready - MAC: %s\n", WiFi.macAddress().c_str());
}

void checkButton() {
    int s = digitalRead(BTN_BOOT);
    uint32_t now = millis();

    if (s == LOW && lastBtnState == HIGH) {
        pressMs = now;
        handledPress = false;
    }
    if (s == HIGH && lastBtnState == LOW) {
        uint32_t held = now - pressMs;
        if (!handledPress && held >= 50 && held < 2000) {
            espNowEnabled = !espNowEnabled;
            Serial.printf("[BTN] ESP-NOW %s\n", espNowEnabled ? "ENABLED" : "DISABLED");
            if (!espNowEnabled) setLED(false);
        }
        handledPress = true;
    }
    lastBtnState = s;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== ESP32-C3 SuperMini - CAN to ESP-NOW Bridge ===");

    pinMode(LED_PIN, OUTPUT);
    setLED(false);
    pinMode(BTN_BOOT, INPUT_PULLUP);

    initCAN();
    initESPNow();
    Serial.println("Listening for CAN frames... tap BOOT to toggle ESP-NOW");
}

void loop() {
    checkButton();
    uint32_t now = millis();

    twai_message_t msg;
    esp_err_t r = twai_receive(&msg, pdMS_TO_TICKS(50));

    if (r == ESP_OK && !msg.rtr) {
        rxCount++;

        if (espNowEnabled) {
            CanFrame frame;
            frame.id = msg.identifier;
            frame.dlc = msg.data_length_code;
            memset(frame.data, 0, 8);
            memcpy(frame.data, msg.data, msg.data_length_code > 8 ? 8 : msg.data_length_code);

            esp_err_t s = esp_now_send(broadcast, (uint8_t *)&frame, sizeof(frame));
            bool ok = (s == ESP_OK);

            txCount++;
            if (!ok) txFail++;
            else lastActivityMs = now;

            Serial.printf("[RX %lu -> TX %lu] ID=0x%03lX [%d] ",
                          (unsigned long)rxCount, (unsigned long)txCount,
                          (unsigned long)msg.identifier, msg.data_length_code);
            for (int i = 0; i < msg.data_length_code; i++) Serial.printf("%02X ", msg.data[i]);
            Serial.printf("NOW:%s\n", ok ? "OK" : "FAIL");
        } else {
            Serial.printf("[RX %lu (NOW OFF)] ID=0x%03lX\n",
                          (unsigned long)rxCount, (unsigned long)msg.identifier);
        }
    }

    if (espNowEnabled && (now - lastActivityMs) < ACTIVITY_TIMEOUT) {
        if (now - lastLedToggle >= LED_FLASH_INTERVAL) {
            lastLedToggle = now;
            setLED(!ledState);
        }
    } else {
        if (ledState) setLED(false);
    }
}
