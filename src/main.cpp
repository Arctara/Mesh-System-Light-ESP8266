/*
    PINOUT
      PIN 4 = Digital Sensor
      PIN A0 = Analog Sensor
      PIN 14 = LED 1
      PIN 12 = LED 2
*/

//$ Include Library
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

//$ Access Point Configuration
#define WIFI_SSID "ALiVe_AP"
#define WIFI_PASS "LeTS_ALiVe"

int sensorReading;
String reading, prevReading;
unsigned long previousMillis = 0;
unsigned long lastScan = 0;
const long interval = 3102;
// const long interval = 10268;

WebSocketsClient webSocket;

DynamicJsonDocument data(1024);
DynamicJsonDocument receivedData(1024);

//* Device Name
const String deviceName = "sensor-1";
const String sensorType = "lightSensor";
const String centerName = "center";

void sendData();
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  webSocket.begin("192.168.5.1", 80, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastScan >= 5000) {
    lastScan = millis();

    if (webSocket.isConnected()) {
      Serial.println("WebSocket Connected");
    } else {
      Serial.println("Connecting (Please Connect)");
      webSocket.begin("192.168.5.1", 80, "/ws");
    }
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println("Sending Data");

    sensorReading = analogRead(A0);

    if (sensorReading <= 448) {
      reading = "Siang";
      Serial.println("Siang");
    } else if (sensorReading > 448 && sensorReading < 570) {
      reading = "Pagi/Sore";
      Serial.println("Pagi/Sore");
    } else if (sensorReading >= 570) {
      reading = "Malam";
      Serial.println("Malam");
    }

    if (reading != prevReading) {
      sendData();
    }
  }
  prevReading = reading;

  webSocket.loop();
}

void sendData() {
  data["from"] = deviceName;
  data["sensorType"] = sensorType;
  data["to"] = centerName;
  data["data"] = reading;
  // data["data"] = sensorReading;
  String msg;
  serializeJson(data, msg);
  webSocket.sendTXT(msg);
  Serial.println("Data sent!");
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    deserializeJson(receivedData, payload);

    String myData;
    serializeJson(receivedData, myData);
    String from = receivedData["from"].as<String>();
    String to = receivedData["to"].as<String>();
    String condition = receivedData["condition"].as<String>();

    Serial.println(myData);
    Serial.println(from);
    Serial.println(to);
    Serial.println(condition);
  }
}
