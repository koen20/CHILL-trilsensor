#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <arduinoFFT.h>

arduinoFFT FFT = arduinoFFT();
#define SAMPLES 512              //Must be a power of 2 was 512 of 1024
#define SAMPLING_FREQUENCY 1600  //Hz. Determines maximum frequency that can be analysed by the FFT.


unsigned int sampling_period_us;
unsigned long microseconds;

// Scaled values for each axis
float scaledX, scaledY, scaledZ;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

char ssid[] = "Henk de Router";
char pass[] = "n@tw@rcd66SXf&";
const char* mqttServer = "20.224.255.243";
int port = 1883;
const char* mqttUsername = "chill";
const char* mqttPassword = "h7pDQ2k8uhxGfoxUQsh";

void setup(void) {
  Serial.begin(115200);
  if (!accel.begin()) {
    Serial.println("No ADXL345 sensor detected.");
    while (1)
      ;
  }
  accel.setDataRate(ADXL345_DATARATE_1600_HZ);
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
  fftLoop();
  int status;
  status = WiFi.status();
  if (status == WL_DISCONNECTED || status == WL_CONNECTION_LOST) {
    while (status != WL_CONNECTED) {
      Serial.println("Disconnected");
      status = WiFi.begin(ssid, pass);
      delay(1000);
    }
  }
}

void fftLoop() {
  sensors_event_t event;
  double vRealADC[SAMPLES];
  double vReal[SAMPLES];
  double vImag[SAMPLES];
  String data = "";
  double avg = 0;
  int count = 0;

  /*SAMPLING                zzzzzzzzzzzzzzzzzzzzzzzzzz*/
  for (int i = 0; i < SAMPLES; i++) {
    accel.getEvent(&event);
    microseconds = micros();

    vReal[i] = event.acceleration.z;
    vImag[i] = 0;
    count = count + 1;
    avg = avg + event.acceleration.z;
    while (micros() < (microseconds + sampling_period_us)) {
    }
  }
  avg = avg / count;
  bool above = false;

  float vibration = 0;
  float start = micros();

  for (int i = 0; i < sizeof(vReal); i++) {
    if (vReal[i] > avg && !above) {
      above = true;
      vibration = vibration + 1;
    } else if (vReal[i] && above) {
      above = false;
    }
  }

  vibration = vibration / ((micros() - start) / 1000000);

  mqttClient.beginMessage("sensors/trilsensor/test");
  mqttClient.print(String(vibration));
  mqttClient.endMessage();


  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peakz = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  /*SAMPLING                xxxxxxxxxxxxxxxxxxxxxxxxx*/
  for (int i = 0; i < SAMPLES; i++) {
    accel.getEvent(&event);
    microseconds = micros();

    vReal[i] = event.acceleration.x;
    vImag[i] = 0;
    Serial.println(event.acceleration.x);

    while (micros() < (microseconds + sampling_period_us)) {
    }
  }
  Serial.println("end");

  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peakx = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  /*SAMPLING                yyyyyyyyyyyyyyyyyyyyyyyyyyy*/
  for (int i = 0; i < SAMPLES; i++) {
    accel.getEvent(&event);
    microseconds = micros();

    vReal[i] = event.acceleration.y;
    vImag[i] = 0;

    while (micros() < (microseconds + sampling_period_us)) {
    }
  }


  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peaky = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  /*PRINT RESULTS*/
  //Serial.println(peakx);     //Print out what frequency is the most dominant.
  mqttClient.beginMessage("sensors/trilsensor");
  mqttClient.print(String(peakx) + ";" + String(peaky) + ";" + String(peakz));
  mqttClient.endMessage();
  //Serial.print(";");
  //Serial.println(count);

  //for (int i = 0; i < (SAMPLES / 2); i++) {
  /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/

  //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
  //Serial.print(" ");
  //Serial.println(vReal[i], 1);  //View only this line in serial plotter to visualize the bins
  //}
}

void publish() {
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
  // Create a random client ID
  String clientId = "ArduinoUno-";
  clientId += String(random(0xffff), HEX);
  // Attempt to connect
  if (mqttClient.connect(mqttServer, port)) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.connectError());
    Serial.println(" try again in 5 seconds");
    delay(5000);
  }
}