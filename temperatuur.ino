#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

char ssid[] = "";
char pass[] = "";
const char* mqttServer = "20.224.255.243";
int port = 1883;
const char* mqttUsername = "chill";
const char* mqttPassword = "h7pDQ2k8uhxGfoxUQsh";
const char topic1[] = "Temperatuur";

//set interval for sending messages (milliseconds)
const long interval = 8000;
unsigned long previousMillis = 0;

void setup(void) {
  Serial.begin(9600);
  sensors.begin();
  Serial.println("Attempting to connect to ");
  Serial.print(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println("Connected");
  mqttClient.setUsernamePassword(mqttUsername, mqttPassword);
}
void loop(void) {
  mqttLoop();
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("C");

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    // save the last time a message was sent
    previousMillis = currentMillis;

    Serial.print("Sending message to topic: ");
    Serial.println(topic1);
    Serial.println(temperature);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic1);
    mqttClient.print(temperature);
    mqttClient.endMessage();
    Serial.println();
  }
  delay(1500);
}

void mqttLoop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.poll();
}

void reconnect() {
  // Loop until we're reconnected
  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(mqttServer, port)) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.connectError());
    Serial.println(" try again in 5 seconds");
    delay(5000);
  }
}
