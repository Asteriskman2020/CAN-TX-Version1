// ============================================================================
//  ESP32-C3 SuperMini - CAN to ESP-NOW Bridge + Web Portal (v2)
//  ----------------------------------------------------------------------------
//  v2 adds a built-in web portal for WiFi setup and live CAN frame view,
//  on top of the v1 CAN-to-ESP-NOW bridging.
//
//  Features:
//    - Reads CAN frames via TJA1050 (TX=GP20, RX=GP21) at 500 kbps
//    - Forwards each frame as ESP-NOW broadcast packet
//    - Hosts a captive AP 'CANBridge-Setup' (open) at 192.168.4.1
//    - Web page allows:
//        * WiFi network scan + connect (credentials saved in NVS)
//        * Live table of last 25 CAN frames (ID, DLC, data, age)
//        * RX / TX counters with a Clear button
//    - BOOT button (GP9) toggles ESP-NOW relay on/off
//    - Built-in LED (GP8) flashes rapidly when relaying CAN traffic
//
//  Hardware:
//    - ESP32-C3 SuperMini
//    - TJA1050 CAN transceiver
//
//  Usage:
//    1. Power up the SuperMini
//    2. From phone/laptop, connect to WiFi 'CANBridge-Setup' (no password)
//    3. Open http://192.168.4.1
//    4. (Optional) scan and connect to your local WiFi to also access the
//       portal via STA IP. ESP-NOW continues to work in AP+STA mode.
// ============================================================================
// ESP32-C3 SuperMini - CAN to ESP-NOW Bridge with Web Portal
// - Reads CAN frames via TJA1050
// - Forwards each frame as ESP-NOW broadcast
// - Hosts a WiFi AP + web portal for config and live frame view
// - WiFi STA credentials saved to NVS via the portal

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <WebServer.h>
#include "driver/twai.h"

// ---- Pins ----
#define BTN_BOOT  9
#define LED_PIN   8
#define CAN_TX    20
#define CAN_RX    21

// ---- Globals ----
struct __attribute__((packed)) CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};
static uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

WebServer server(80);
Preferences prefs;

static uint32_t rxCount = 0;
static uint32_t txCount = 0;
static uint32_t txFail = 0;
static volatile bool espNowEnabled = true;

struct LoggedFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint32_t timestamp;
};
static const int LOG_SIZE = 25;
static LoggedFrame logBuf[LOG_SIZE];
static int logIdx = 0;
static int logFill = 0;

static String currentSSID = "";
static bool wifiConnected = false;

static int lastBtnState = HIGH;
static uint32_t pressMs = 0;
static bool handledPress = false;
static uint32_t lastLedToggle = 0;
static uint32_t lastActivityMs = 0;
static bool ledState = false;

void setLED(bool on) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);
    ledState = on;
}

void initCAN() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
    Serial.println("[CAN] Started @ 500 kbps");
}

void initESPNow() {
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

void connectSavedWiFi() {
    prefs.begin("wifi", true);
    String ssid = prefs.getString("ssid", "");
    String pass = prefs.getString("pass", "");
    prefs.end();
    if (ssid.length() == 0) return;

    Serial.printf("[WIFI] Connecting to %s...\n", ssid.c_str());
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) delay(200);

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
        currentSSID = ssid;
        wifiConnected = true;
    } else {
        Serial.println("[WIFI] STA connect failed");
    }
}

const char* PAGE_HTML PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>CAN Bridge</title>
<style>
body{font-family:system-ui,sans-serif;background:#0d1117;color:#e6edf3;margin:0;padding:16px;max-width:920px;margin:0 auto}
h1{color:#58a6ff;margin:8px 0}h2{color:#3fb950;margin-top:24px;font-size:16px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin:10px 0}
button{background:#238636;color:#fff;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-size:13px;margin:4px}
button:hover{background:#2ea043}
button.danger{background:#da3633}button.danger:hover{background:#f85149}
button.scan{background:#1f6feb}button.scan:hover{background:#388bfd}
input{background:#0d1117;border:1px solid #30363d;color:#e6edf3;padding:6px 10px;border-radius:6px;width:180px;font-size:13px;margin:2px}
table{width:100%;border-collapse:collapse;font-family:monospace;font-size:12px}
th,td{padding:4px 8px;text-align:left;border-bottom:1px solid #21262d}
th{color:#8b949e}.id{color:#fd7e14}.data{color:#e6edf3}
.stat{display:inline-block;margin-right:18px;font-family:monospace}.stat b{color:#3fb950}
.netlist{max-height:180px;overflow-y:auto}
.net{cursor:pointer;padding:5px;border-radius:4px}.net:hover{background:#21262d}
.status{padding:3px 8px;border-radius:4px;font-size:12px;display:inline-block}
.status.ok{background:#23863666;color:#3fb950}.status.no{background:#da363366;color:#f85149}
</style></head><body>
<h1>CAN-to-ESP-NOW Bridge</h1>
<div class="card"><h2>WiFi Setup</h2>
<div id="wifi-status">Loading...</div>
<div style="margin-top:8px"><button class="scan" onclick="scanWiFi()">Scan Networks</button>
<div id="netlist" class="netlist" style="margin-top:6px"></div></div>
<div style="margin-top:10px">SSID: <input id="ssid" placeholder="Network">
Password: <input id="pass" type="password" placeholder="Password">
<button onclick="connectWiFi()">Connect & Reboot</button></div></div>
<div class="card"><h2>CAN Frames
<span class="stat">RX: <b id="rxc">0</b></span>
<span class="stat">TX: <b id="txc">0</b></span>
<button class="danger" onclick="clearLog()">Clear Counter</button></h2>
<table><thead><tr><th>Age</th><th>ID</th><th>DLC</th><th>Data</th></tr></thead>
<tbody id="frames"></tbody></table></div>
<script>
async function fetchFrames(){
  try {
    const r=await fetch('/frames');const j=await r.json();
    document.getElementById('rxc').textContent=j.rx;
    document.getElementById('txc').textContent=j.tx;
    document.getElementById('wifi-status').innerHTML=j.wifi.connected?
      '<span class="status ok">Connected: '+j.wifi.ssid+' ('+j.wifi.ip+')</span>':
      '<span class="status no">Not connected (AP mode at '+j.wifi.ap+')</span>';
    let h='';
    for(const f of j.frames){
      const d=f.data.map(b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ');
      h+='<tr><td>'+f.t+'ms</td><td class="id">0x'+f.id.toString(16).toUpperCase().padStart(3,'0')+'</td><td>'+f.dlc+'</td><td class="data">'+d+'</td></tr>';
    }
    document.getElementById('frames').innerHTML=h;
  }catch(e){}
}
async function scanWiFi(){
  document.getElementById('netlist').innerHTML='Scanning...';
  const r=await fetch('/scan');const j=await r.json();
  let h='';for(const n of j) h+='<div class="net" onclick="document.getElementById('ssid').value=''+n.ssid+''">'+n.ssid+' <small style="color:#8b949e">'+n.rssi+'dBm</small></div>';
  document.getElementById('netlist').innerHTML=h||'<i>No networks</i>';
}
async function connectWiFi(){
  const s=document.getElementById('ssid').value;const p=document.getElementById('pass').value;
  if(!s)return alert('Enter SSID');
  await fetch('/connect',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p)});
  alert('Saved. Device rebooting...');
}
async function clearLog(){await fetch('/clear',{method:'POST'});fetchFrames();}
setInterval(fetchFrames,800);fetchFrames();
</script></body></html>)HTML";

void handleRoot() {
    server.send_P(200, "text/html", PAGE_HTML);
}

void handleScan() {
    int n = WiFi.scanNetworks(false, false);
    String j = "[";
    for (int i = 0; i < n; i++) {
        if (i > 0) j += ",";
        String ss = WiFi.SSID(i);
        ss.replace("\"", "\\\"");
        j += "{\"ssid\":\"" + ss + "\",\"rssi\":" + WiFi.RSSI(i) + "}";
    }
    j += "]";
    WiFi.scanDelete();
    server.send(200, "application/json", j);
}

void handleConnect() {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    server.send(200, "text/plain", "OK");
    delay(500);
    ESP.restart();
}

void handleFrames() {
    String j = "{\"rx\":" + String(rxCount) + ",\"tx\":" + String(txCount);
    j += ",\"wifi\":{\"connected\":";
    j += (wifiConnected ? "true" : "false");
    j += ",\"ssid\":\"" + currentSSID + "\",\"ip\":\"" + WiFi.localIP().toString();
    j += "\",\"ap\":\"" + WiFi.softAPIP().toString() + "\"},\"frames\":[";

    int count = logFill < LOG_SIZE ? logFill : LOG_SIZE;
    uint32_t now = millis();
    for (int i = 0; i < count; i++) {
        int idx = (logIdx - 1 - i + LOG_SIZE) % LOG_SIZE;
        LoggedFrame &f = logBuf[idx];
        if (i > 0) j += ",";
        j += "{\"id\":" + String(f.id) + ",\"dlc\":" + String(f.dlc);
        j += ",\"t\":" + String(now - f.timestamp) + ",\"data\":[";
        for (int b = 0; b < f.dlc; b++) {
            if (b > 0) j += ",";
            j += String(f.data[b]);
        }
        j += "]}";
    }
    j += "]}";
    server.send(200, "application/json", j);
}

void handleClear() {
    rxCount = 0;
    txCount = 0;
    txFail = 0;
    logFill = 0;
    logIdx = 0;
    server.send(200, "text/plain", "OK");
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== SuperMini CAN-ESP-NOW Bridge + Web Portal ===");

    pinMode(LED_PIN, OUTPUT);
    setLED(false);
    pinMode(BTN_BOOT, INPUT_PULLUP);

    // WiFi: AP + try saved STA
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("CANBridge-Setup");
    Serial.printf("[WIFI] AP: SSID=CANBridge-Setup, IP=%s\n",
                  WiFi.softAPIP().toString().c_str());
    connectSavedWiFi();

    initESPNow();
    initCAN();

    server.on("/", handleRoot);
    server.on("/scan", handleScan);
    server.on("/connect", HTTP_POST, handleConnect);
    server.on("/frames", handleFrames);
    server.on("/clear", HTTP_POST, handleClear);
    server.begin();
    Serial.println("[HTTP] Server on port 80");
    Serial.println("Connect to AP 'CANBridge-Setup' and open http://192.168.4.1");
}

void loop() {
    server.handleClient();

    // Button: short tap toggles ESP-NOW
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

    // CAN receive + forward
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK && !msg.rtr) {
        rxCount++;

        logBuf[logIdx].id = msg.identifier;
        logBuf[logIdx].dlc = msg.data_length_code;
        memset(logBuf[logIdx].data, 0, 8);
        memcpy(logBuf[logIdx].data, msg.data,
               msg.data_length_code > 8 ? 8 : msg.data_length_code);
        logBuf[logIdx].timestamp = now;
        logIdx = (logIdx + 1) % LOG_SIZE;
        if (logFill < LOG_SIZE) logFill++;

        if (espNowEnabled) {
            CanFrame frame;
            frame.id = msg.identifier;
            frame.dlc = msg.data_length_code;
            memset(frame.data, 0, 8);
            memcpy(frame.data, msg.data,
                   msg.data_length_code > 8 ? 8 : msg.data_length_code);

            esp_err_t r = esp_now_send(broadcast, (uint8_t *)&frame, sizeof(frame));
            if (r == ESP_OK) {
                txCount++;
                lastActivityMs = now;
            } else {
                txFail++;
            }
        }
    }

    // LED rapid flash on activity
    if (espNowEnabled && (now - lastActivityMs) < 300) {
        if (now - lastLedToggle >= 50) {
            lastLedToggle = now;
            setLED(!ledState);
        }
    } else {
        if (ledState) setLED(false);
    }
}

