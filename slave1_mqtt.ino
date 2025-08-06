#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CHANNEL 1
#define ZONE 1
#define TRIG_PIN 4
#define ECHO_PIN 5
#define MAX_TTL 3

// LED Pins
#define LED_VALVE 15    // Green LED: Valve ON/OFF
#define LED_SEND  16     // Red LED: Blink on data sent


const float maxHeight = 100.0;
const float thresholdGap = 15.0;
const unsigned long sendInterval = 3000;
const unsigned long autoCloseDelay = 3000;
const float tankArea = 500.0;

uint8_t masterMac[] = {0x10, 0x06, 0x1C, 0xD7, 0x00, 0x28};
uint8_t broadcastAddress[] = {0x30, 0xED, 0xA0, 0x27, 0x97, 0x3C};
uint8_t self_mac[6];

typedef struct {
  int zone;
  int command;
  int status;
  float flow;
  float level;
  float height;
  uint8_t origin[6];
  uint8_t ttl;
} Message;

Message incomingData, feedbackData;
bool valveOpen = false;
bool pendingAutoClose = false;

unsigned long lastSentTime = 0;
unsigned long autoCloseStartTime = 0;

float lastLevel = 0.0;

float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return lastLevel;

  float distance = duration * 0.034 / 2.0;
  if (distance > maxHeight || distance <= 0) return lastLevel;

  return maxHeight - distance;
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Message received;
  memcpy(&received, data, sizeof(Message));

  if (memcmp(info->src_addr, WiFi.macAddress().c_str(), 6) == 0) return;

  if (received.ttl > 0 && received.zone != ZONE) {
    received.ttl--;
    Serial.print("Relaying packet with updated TTL = ");
    Serial.println(received.ttl);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);
    Serial.printf("🔁 Relayed packet from MAC: %s\n", macStr);
    esp_now_send(broadcastAddress, (uint8_t *)&received, sizeof(Message));

    return;
  }
  else{
    Serial.println("TTL expired. Dropping packet.");
  }

  if (received.zone == ZONE) {
  incomingData = received;  // ✅ Critical line to fix the bug

  if (incomingData.command == 1 && !valveOpen) {
    Serial.println("🟢 Opening Valve");
    valveOpen = true;
    pendingAutoClose = false;
    digitalWrite(LED_VALVE, HIGH);
  } else if (incomingData.command == 0 && valveOpen) {
    Serial.println("🔴 Closing Valve");
    valveOpen = false;
    pendingAutoClose = false;
    digitalWrite(LED_VALVE, LOW);
  }
}


  }


void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Feedback sent" : "❌ Send failed");

  // Blink RED LED for data sent
  digitalWrite(LED_SEND, HIGH);
  delay(50);
  digitalWrite(LED_SEND, LOW);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  // LED Setup
  pinMode(LED_VALVE, OUTPUT);
  pinMode(LED_SEND, OUTPUT);

  digitalWrite(LED_VALVE, LOW);
  digitalWrite(LED_SEND, LOW);
 

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMac, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  esp_wifi_get_mac(WIFI_IF_STA, self_mac);
  Serial.printf("📡 This node MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                self_mac[0], self_mac[1], self_mac[2],
                self_mac[3], self_mac[4], self_mac[5]);

  Serial.println("✅ Slave Node Ready (Zone 0)");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSentTime > sendInterval) {
    lastSentTime = now;

    float level = readUltrasonicCM();

    float deltaLevel = level - lastLevel;
    float deltaTimeMin = (sendInterval / 1000.0) / 60.0;
    float simulatedFlow = (deltaLevel * tankArea) / (1000.0 * deltaTimeMin);
    if (simulatedFlow < 0) simulatedFlow = 0.0;
    lastLevel = level;

    digitalWrite(LED_VALVE, valveOpen ? HIGH : LOW);

    // auto cclose logic
    if (valveOpen && level >= (maxHeight - thresholdGap)) {
      if (!pendingAutoClose) {
        Serial.println("Threshold reached. Starting auto-close timer");
        pendingAutoClose = true;
        autoCloseStartTime = now;
      }
    }

    if (pendingAutoClose && (now - autoCloseStartTime >= autoCloseDelay)) {
      Serial.println("Auto-close timer expired. Closing valve.");
      valveOpen = false;
      pendingAutoClose = false;

      feedbackData.zone = ZONE;
      feedbackData.command = 2;
      feedbackData.status = 2;
      feedbackData.level = level;
      feedbackData.flow = 0.0;
      feedbackData.height = maxHeight;
      feedbackData.ttl = MAX_TTL;
      esp_wifi_get_mac(WIFI_IF_STA, feedbackData.origin);
      esp_now_send(NULL, (uint8_t *)&feedbackData, sizeof(feedbackData));
    }

    // Send telemetry
    feedbackData.zone = ZONE;
    feedbackData.command = 0;
    feedbackData.status = valveOpen ? 1 : 2;
    feedbackData.level = level;
    feedbackData.flow = valveOpen ? simulatedFlow : 0.0;
    feedbackData.height = maxHeight;
    feedbackData.ttl = MAX_TTL;
    esp_wifi_get_mac(WIFI_IF_STA, feedbackData.origin);
    esp_now_send(NULL, (uint8_t *)&feedbackData, sizeof(feedbackData));

    Serial.printf("📤 Sent | Zone: %d | Level: %.2f cm | Flow: %.2f L/min | Valve: %s\n",
                  ZONE, level, feedbackData.flow, valveOpen ? "OPEN" : "CLOSED");

    Serial.printf("📡 This node MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  self_mac[0], self_mac[1], self_mac[2],
                  self_mac[3], self_mac[4], self_mac[5]);
  }
}
