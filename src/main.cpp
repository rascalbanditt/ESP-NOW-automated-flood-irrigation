#include <Arduino.h>
#include "utilities.h" //

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// debug and Serial
#define SerialMon Serial

// GSM setup
const char apn[] = "jazzconnect.mobilinkworld.com";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char *broker = "mqtt.eu.thingsboard.cloud";
const char *accessToken = "r3unTBIz12Tc5LoIiga9";
int valveStatuses[3] = {0, 0, 0}; // status for each zone (0 = unknown, 1 = open, 2 = closed)

TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

// ESP-NOW setup
#define CHANNEL 1
uint8_t slaves[][6] = {
    {0x30, 0xED, 0xA0, 0x27, 0x97, 0x3C},
    {0x30, 0xED, 0xA0, 0x27, 0x44, 0x98},
    {0x30, 0xED, 0xA0, 0x27, 0x06, 0x6C}};

typedef struct
{
  int zone;
  int command;
  int status;
  float flow;
  float level;
  float height;
  int ttl;
  uint8_t origin[6];
} Message;

Message outgoing, incoming;

// esp now callbacks
void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
  memcpy(&incoming, data, sizeof(incoming));
  Serial.printf("Received from zone %d, command=%d, status=%d, flow=%.2f, level=%.2f height=%.2f\n",
                incoming.zone, incoming.command, incoming.status, incoming.flow, incoming.level, incoming.ttl);
  char originStr[18];
  sprintf(originStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          incoming.origin[0], incoming.origin[1], incoming.origin[2],
          incoming.origin[3], incoming.origin[4], incoming.origin[5]);
  Serial.printf("📡 Origin MAC: %s\n", originStr);

  // store status
  if (incoming.zone >= 0 && incoming.zone < 3)
  {
    valveStatuses[incoming.zone] = incoming.status;
  }

  JsonDocument full;

  full["zone"] = incoming.zone;
  full["command"] = incoming.command;
  full["status"] = incoming.status;
  full["flow"] = incoming.flow;
  full["level"] = incoming.level;
  full["height"] = incoming.height;
  full["ttl"] = incoming.ttl;
  full["origin"] = originStr;

  String payload;
  serializeJson(full, payload);
  mqtt.publish("v1/devices/me/telemetry", payload.c_str());

  // publish flat key for led indicator  "zone_0_status": 1
  JsonDocument flat;
  flat["zone_" + String(incoming.zone) + "_status"] = incoming.status;
  flat["zone_" + String(incoming.zone) + "_flow"] = incoming.flow;
  flat["zone_" + String(incoming.zone) + "_level"] = incoming.level;
  flat["zone_" + String(incoming.zone) + "_height"] = incoming.height;
  flat["zone_" + String(incoming.zone) + "_origin"] = originStr; // now works
  String flatPayload;
  serializeJson(flat, flatPayload);
  mqtt.publish("v1/devices/me/telemetry", flatPayload.c_str());
  yield();
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);

  unsigned long now = millis(); // timestamp

  if (status == ESP_NOW_SEND_SUCCESS)
  {
    SerialMon.printf("[%lu ms] Sent to %s successfully\n", now, macStr);
  }
  else
  {
    SerialMon.printf(" [%lu ms] Failed to send to %s\n", now, macStr);
  }
}

void incomingRpc(char *topic, byte *payload, unsigned int length)
{
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err)
  {
    SerialMon.print("JSON error: ");
    SerialMon.println(err.c_str());
    return;
  }

  const char *method = doc["method"];
  JsonVariant params = doc["params"];

  if (strcmp(method, "controlValve") == 0 && params["zone"].is<int>() && params["cmd"].is<const char *>())
  {
    int zone = params["zone"];
    const char *cmd = params["cmd"];

    if (zone < 0 || zone >= sizeof(slaves) / sizeof(slaves[0]))
    {
      SerialMon.println("Invalid zone index received!");
      return;
    }

    outgoing.zone = zone;
    outgoing.command = (strcmp(cmd, "open") == 0) ? 1 : 0;
    outgoing.status = -1;
    outgoing.ttl = 2;

    esp_err_t result = esp_now_send(slaves[zone], (uint8_t *)&outgoing, sizeof(outgoing));
    SerialMon.println(result == ESP_OK ? "Command sent" : "Send failed");
  }

  else if (strcmp(method, "getValveStatus") == 0 && params["zone"].is<int>())
  {
    int zone = params["zone"];
    if (zone < 0 || zone >= 3)
    {
      SerialMon.println("Invalid zone for status check!");
      return;
    }

    JsonDocument response;
    response["zone"] = zone;
    response["status"] = valveStatuses[zone];

    String payload;
    serializeJson(response, payload);
    mqtt.publish(topic, payload.c_str()); // reply back to caller
    SerialMon.printf("Replied with valve status for zone %d: %d\n", zone, valveStatuses[zone]);
  }

  else
  {
    SerialMon.println("Invalid RPC payload");
  }
}

// mqtt implementation
bool mqttConnect()
{
  SerialMon.print("Connecting to MQTT...");
  bool status = mqtt.connect("esp32Client", accessToken, NULL);
  if (!status)
  {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe("v1/devices/me/rpc/request/+");
  mqtt.publish("v1/devices/me/telemetry", "{\"msg\":\"boot complete\"}");
  return true;
}

// ====== Setup ======
void setup()
{
  SerialMon.begin(115200);
  delay(3000);

#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

#ifdef MODEM_RESET_PIN
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(3000);

  SerialMon.println("Initializing modem...");
  if (!modem.init())
  {
    SerialMon.println("Modem init failed.");
    return;
  }

  if (!modem.waitForNetwork())
  {
    SerialMon.println("Network not found.");
    return;
  }

  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    SerialMon.println("GPRS failed.");
    return;
  }

  SerialMon.println("GPRS connected.");

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(incomingRpc);
  mqttConnect();

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK)
  {
    SerialMon.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSent);

  for (auto &mac : slaves)
  {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }
}

// loop
void loop()
{
  if (!modem.isNetworkConnected())
    modem.waitForNetwork();
  if (!modem.isGprsConnected())
    modem.gprsConnect(apn, gprsUser, gprsPass);
  if (!mqtt.connected())
    mqttConnect();
  mqtt.loop();

  JsonDocument doc;
  doc["status"] = "online";
  doc["device"] = "master";
  String buf;
  serializeJson(doc, buf);
  mqtt.publish("v1/devices/me/telemetry", buf.c_str());

  delay(5000);
  yield();
}
