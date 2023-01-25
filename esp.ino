#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <ArduinoFFT.h>

Adafruit_ADXL345 accel = Adafruit_ADXL345();
ArduinoFFT FFT = ArduinoFFT();

const int sampleSize = 128;
float x[sampleSize];
float y[sampleSize];
float z[sampleSize];

void setup() {
  Wire.begin();
  Serial.begin(115200);
  accel.begin();
}

void loop() {
  // Read the x, y, and z axis acceleration values into an array
  for (int i = 0; i < sampleSize; i++) {
    x[i] = accel.readFloatAccelX();
    y[i] = accel.readFloatAccelY();
    z[i] = accel.readFloatAccelZ();
    delay(10);
  }

  // Perform FFT on the x-axis values
  FFT.Windowing(x, sampleSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(x, sampleSize, FFT_FORWARD);
  FFT.ComplexToMagnitude(x, sampleSize);

  // Perform FFT on the y-axis values
  FFT.Windowing(y, sampleSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(y, sampleSize, FFT_FORWARD);
  FFT.ComplexToMagnitude(y, sampleSize);

  // Perform FFT on the z-axis values
  FFT.Windowing(z, sampleSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(z, sampleSize, FFT_FORWARD);
  FFT.ComplexToMagnitude(z, sampleSize);

  // Print the FFT results to the serial monitor
  for (int i = 0; i < sampleSize / 2; i++) {
    Serial.print("X: ");
    Serial.print(x[i]);
    Serial.print("  Y: ");
    Serial.print(y[i]);
    Serial.print("  Z: ");
    Serial.println(z[i]);
  }
}
