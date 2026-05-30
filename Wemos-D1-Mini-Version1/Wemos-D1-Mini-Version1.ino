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
//  LED behaviour (priority high -> low):
//    1. Solid ON          during the 1 s garage relay pulse
//    2. Rapid flash 20Hz  when a watched frame arrives:
//                           - Benz speak frame 0x3F6
//                           - OBD2 RPM reply     (PID 0x0C on 0x7E8..0x7EF)
//                           - OBD2 Speed reply   (PID 0x0D)
//                           - OBD2 Coolant reply (PID 0x05)
//    3. Slow flash 10Hz   on any ESP-NOW relay activity
//    4. Off               idle
//
//  Pins:
//        CAN_TX       = GPIO4   (to TJA1050 CTX)
//        CAN_RX       = GPIO5   (to TJA1050 CRX)
//        GARAGE_RELAY = GPIO16  (moved from GPIO5 due to CAN_RX conflict)
//        LED          = GPIO2   (D4, active-LOW)
//        BTN_BOOT     = GPIO0
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
#define CAN_TX            4     // to TJA1050 CTX
#define CAN_RX            5     // to TJA1050 CRX
#define GARAGE_RELAY_PIN 16     // moved from GPIO5 (CAN_RX needed it)
#define LED_PIN           2     // active-LOW
#define BTN_BOOT          0

// ---- Garage opener tuning ----
#define BENZ_SWC_CAN_ID         0x3F6   // 1014 - VCLEFT_switchStatus (Speak btn bit 12)
#define FORD_CLU11_CAN_ID       0x09A   // 154  - CLU11 (Cruise btn bit 56)
#define GARAGE_HOLD_MS          3000
#define GARAGE_PULSE_MS         1000
#define GARAGE_REARM_MS         2000

// ---- Watched-frame indicator (rapid LED flash on detection) ----
#define OBD_RPM_PID         0x0C
#define OBD_SPEED_PID       0x0D
#define OBD_COOLANT_PID     0x05
#define WATCHED_WINDOW_MS   150
#define WATCHED_TOGGLE_MS    25      // ~20 Hz flash

// ---- ESP-NOW payload ----
struct __attribute__((packed)) CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

// ---- Globals ----
static uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool    espNowEnabled = true;

// Garage state machine
static bool     btnHeld          = false;
static uint32_t btnPressStartMs  = 0;
static uint32_t garagePulseStart = 0;
static uint32_t lastTriggerMs    = 0;

// LED activity tracking
static uint32_t lastLedToggle  = 0;
static uint32_t lastActivityMs = 0;     // any ESP-NOW relay
static uint32_t lastWatchedMs  = 0;     // RPM / Speed / Coolant / Speak frames
static bool     ledState       = false;

// BOOT button
static int      lastBtnState = HIGH;
static uint32_t pressMs      = 0;
static bool     handledPress = false;

// ---- LED (active-LOW) ----
static void setLED(bool on) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);
    ledState = on;
}

// ---- CAN (LISTEN_ONLY: silent on the bus) ----
static void initCAN() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
}

// ---- ESP-NOW ----
static void initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    if (esp_now_init() != ESP_OK) return;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

// ---- Garage opener ----
// Bit numbering: raw_ul = d[0..7] little-endian -> bit N = byte (N/8) bit (N%8)
//   Benz speak  (bit 12) -> data[1] >> 4 & 1
//   Ford cruise (bit 56) -> data[7] & 1
static void watchGarageButton(const twai_message_t& msg) {
    bool pressed = false;

    if (msg.identifier == BENZ_SWC_CAN_ID && msg.data_length_code >= 2) {
        pressed = (msg.data[1] >> 4) & 0x01;
    } else if (msg.identifier == FORD_CLU11_CAN_ID && msg.data_length_code >= 8) {
        pressed = msg.data[7] & 0x01;
    } else {
        return;
    }

    uint32_t now = millis();
    if (pressed && !btnHeld) {
        btnHeld = true;
        btnPressStartMs = now;
    } else if (!pressed && btnHeld) {
        btnHeld = false;
        btnPressStartMs = 0;
    }
}

// Returns true if this frame is one we want to flash the LED rapidly for:
//   - Benz speak frame 0x3F6
//   - OBD2 mode-1 reply on 0x7E8..0x7EF carrying RPM / Speed / Coolant PID
static bool isWatchedFrame(const twai_message_t& msg) {
    if (msg.identifier == BENZ_SWC_CAN_ID) return true;
    if (msg.identifier >= 0x7E8 && msg.identifier <= 0x7EF &&
        msg.data_length_code >= 3 && msg.data[1] == 0x41) {
        uint8_t pid = msg.data[2];
        return pid == OBD_RPM_PID || pid == OBD_SPEED_PID || pid == OBD_COOLANT_PID;
    }
    return false;
}

static void garageStart() {
    if (garagePulseStart != 0) return;
    if (millis() - lastTriggerMs < GARAGE_REARM_MS && lastTriggerMs) return;
    digitalWrite(GARAGE_RELAY_PIN, HIGH);
    setLED(true);
    garagePulseStart = millis();
}

static void garageService() {
    if (!garagePulseStart) return;
    if (millis() - garagePulseStart >= GARAGE_PULSE_MS) {
        digitalWrite(GARAGE_RELAY_PIN, LOW);
        setLED(false);
        lastTriggerMs = millis();
        garagePulseStart = 0;
    }
}

void setup() {
    // No Serial.begin() — keep UART0 free; all feedback is via LED + relay

    pinMode(GARAGE_RELAY_PIN, OUTPUT);
    digitalWrite(GARAGE_RELAY_PIN, LOW);
    pinMode(LED_PIN, OUTPUT);
    setLED(false);
    pinMode(BTN_BOOT, INPUT_PULLUP);

    initESPNow();
    initCAN();
}

void loop() {
    // ---- BOOT button: short = ESP-NOW toggle, long = manual garage trigger ----
    int s = digitalRead(BTN_BOOT);
    uint32_t now = millis();
    if (s == LOW && lastBtnState == HIGH) {
        pressMs = now; handledPress = false;
    }
    if (s == LOW && !handledPress && (now - pressMs) >= 2000) {
        garageStart();
        handledPress = true;
    }
    if (s == HIGH && lastBtnState == LOW) {
        uint32_t held = now - pressMs;
        if (!handledPress && held >= 50 && held < 2000) {
            espNowEnabled = !espNowEnabled;
            if (!espNowEnabled && !garagePulseStart) setLED(false);
        }
        handledPress = true;
    }
    lastBtnState = s;

    // ---- Garage hold timer ----
    if (btnHeld && (now - btnPressStartMs) >= GARAGE_HOLD_MS) {
        garageStart();
        btnHeld = false; btnPressStartMs = 0;
    }
    garageService();

    // ---- CAN RX -> watch garage btn, mark watched, ESP-NOW relay ----
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK && !msg.rtr) {
        watchGarageButton(msg);
        if (isWatchedFrame(msg)) lastWatchedMs = now;

        if (espNowEnabled) {
            CanFrame frame;
            frame.id  = msg.identifier;
            frame.dlc = msg.data_length_code;
            memset(frame.data, 0, 8);
            memcpy(frame.data, msg.data,
                   msg.data_length_code > 8 ? 8 : msg.data_length_code);
            esp_err_t r = esp_now_send(broadcast, (uint8_t*)&frame, sizeof(frame));
            if (r == ESP_OK) lastActivityMs = now;
        }
    }

    // ---- LED priority: garage > watched-rapid > activity-slow > off ----
    if (!garagePulseStart) {
        bool watched  = lastWatchedMs  && (now - lastWatchedMs)  < WATCHED_WINDOW_MS;
        bool activity = espNowEnabled  && lastActivityMs && (now - lastActivityMs) < 300;
        if (watched) {
            if (now - lastLedToggle >= WATCHED_TOGGLE_MS) {
                lastLedToggle = now;
                setLED(!ledState);
            }
        } else if (activity) {
            if (now - lastLedToggle >= 50) {
                lastLedToggle = now;
                setLED(!ledState);
            }
        } else if (ledState) {
            setLED(false);
        }
    }
}
