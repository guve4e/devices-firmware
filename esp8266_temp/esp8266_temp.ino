#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

#define DHTPIN 4  // GPIO4 (D2)
#define DHTTYPE DHT22
#define EEPROM_SIZE 512

const char* ssid = "Housenet";
const char* password = "tvoitasem4ica";

const char* mqttServer = "devicebroker.ddns.net";
const int mqttPort = 1883;
const char* deviceId = "esp8266-temp-sensor-001";

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

struct Settings {
  bool persistent = true;
  unsigned long intervalSeconds = 2;
  bool showHumidity = false;
  float temperatureThreshold = 0.3;
  float humidityThreshold = 1.0;
} settings;

unsigned long lastSend = 0;
float lastTemperature = -1000;
float lastHumidity = -1000;

void saveSettings() {
  EEPROM.put(0, settings);
  EEPROM.commit();
  Serial.printf("💾 Settings saved: interval=%lus, persistent=%s, showHumidity=%s, tempThresh=%.2f, humThresh=%.2f\n",
                settings.intervalSeconds,
                settings.persistent ? "true" : "false",
                settings.showHumidity ? "true" : "false",
                settings.temperatureThreshold,
                settings.humidityThreshold);
}

void loadSettings() {
  EEPROM.get(0, settings);
  if (settings.intervalSeconds < 1 || settings.intervalSeconds > 3600) {
    Serial.println("⚠️ Invalid interval. Resetting to defaults.");
    settings = Settings();
    saveSettings();
  } else {
    Serial.printf("📤 Settings loaded: interval=%lus, persistent=%s, showHumidity=%s, tempThresh=%.2f, humThresh=%.2f\n",
                  settings.intervalSeconds,
                  settings.persistent ? "true" : "false",
                  settings.showHumidity ? "true" : "false",
                  settings.temperatureThreshold,
                  settings.humidityThreshold);
  }
}

void sendSelfRegistration(bool isHeartbeat = false) {
  StaticJsonDocument<512> doc;
  doc["deviceId"] = deviceId;
  doc["name"] = "ESP8266 Temp Sensor";
  doc["type"] = "sensor";
  doc["firmwareVersion"] = "1.1.0";
  doc["location"] = "garage";
  doc["timestamp"] = millis();
  doc["heartbeat"] = isHeartbeat;

  JsonObject settingsObj = doc.createNestedObject("settings");
  settingsObj["persistent"] = settings.persistent;
  settingsObj["intervalSeconds"] = settings.intervalSeconds;
  settingsObj["showHumidity"] = settings.showHumidity;

  JsonObject threshold = settingsObj.createNestedObject("threshold");
  threshold["temperature"] = settings.temperatureThreshold;
  threshold["humidity"] = settings.humidityThreshold;

  char buffer[512];
  serializeJson(doc, buffer);
  String topic = "devices/" + String(deviceId) + "/status";
  boolean success = client.publish(topic.c_str(), buffer, true);
  Serial.println("📤 Sent self-registration: " + String(buffer));
  if (!success) {
    Serial.println("❌ Failed to publish registration to MQTT.");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📥 MQTT message on topic: ");
  Serial.println(topic);
  payload[length] = '\0';
  Serial.println("📦 Payload: " + String((char*)payload));

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.println("❌ JSON parse error.");
    return;
  }

  bool updated = false;
  if (doc.containsKey("settings")) {
    JsonObject s = doc["settings"];

    if (s.containsKey("persistent")) {
      settings.persistent = s["persistent"];
      updated = true;
    }
    if (s.containsKey("intervalSeconds")) {
      settings.intervalSeconds = s["intervalSeconds"];
      updated = true;
    }
    if (s.containsKey("showHumidity")) {
      settings.showHumidity = s["showHumidity"];
      updated = true;
    }

    if (s.containsKey("threshold")) {
      JsonObject t = s["threshold"];
      if (t.containsKey("temperature")) {
        settings.temperatureThreshold = t["temperature"];
        updated = true;
      }
      if (t.containsKey("humidity")) {
        settings.humidityThreshold = t["humidity"];
        updated = true;
      }
    }
  }

  if (updated) {
    saveSettings();
    sendSelfRegistration();
  }
}

void connectToWiFi() {
  Serial.printf("🔌 Connecting to SSID: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(" ✅ WiFi connected!");
  Serial.print("📡 IP: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.print("🔌 Connecting to MQTT...");
    if (client.connect(deviceId)) {
      Serial.println(" ✅ MQTT connected!");
      client.subscribe(("devices/" + String(deviceId) + "/config").c_str());
      sendSelfRegistration();
    } else {
      Serial.print("❌ MQTT failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setupOTA() {
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  Serial.println("🛠 OTA server started.");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  connectToWiFi();
  dht.begin();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  client.setBufferSize(512); // 🚀 Increase MQTT buffer size
  setupOTA();
}

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();
  httpServer.handleClient();

  unsigned long now = millis();
  unsigned long intervalMs = settings.intervalSeconds * 1000;

  if (now - lastSend >= intervalMs) {
    lastSend = now;

    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    if (isnan(temp)) {
      Serial.println("⚠️ Failed to read temperature.");
      return;
    }

    bool shouldSend = false;

    if (abs(temp - lastTemperature) >= settings.temperatureThreshold) {
      shouldSend = true;
    }

    if (settings.showHumidity && !isnan(hum)) {
      if (abs(hum - lastHumidity) >= settings.humidityThreshold) {
        shouldSend = true;
      }
    }

    if (shouldSend && settings.persistent) {
      StaticJsonDocument<256> payload;
      payload["temperature"] = temp;
      payload["timestamp"] = now;

      if (settings.showHumidity && !isnan(hum)) {
        payload["humidity"] = hum;
      }

      char buffer[256];
      serializeJson(payload, buffer);
      String topic = "devices/" + String(deviceId) + "/data";
      bool success = client.publish(topic.c_str(), buffer);
      Serial.println("📤 Published data: " + String(buffer));
      if (!success) {
        Serial.println("❌ Failed to publish data!");
      }

      lastTemperature = temp;
      lastHumidity = hum;

      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  // Heartbeat every 60 seconds
  static unsigned long lastHeartbeat = 0;
  const unsigned long heartbeatInterval = 60000;
  if (now - lastHeartbeat >= heartbeatInterval) {
    lastHeartbeat = now;
    sendSelfRegistration(true);
  }
}
