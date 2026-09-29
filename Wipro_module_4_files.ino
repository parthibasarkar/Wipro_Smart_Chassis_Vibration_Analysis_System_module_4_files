#include <Wire.h>
#include "arduinoFFT.h"

#define MPU6050_ADDR 0x68
#define LED_PIN 4

const uint16_t samples = 64;
const double samplingFrequency = 100.0;

double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(
  vReal, vImag, samples, samplingFrequency
);

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  Serial.println("Smart Chassis Vibration Monitor");
  Serial.println("MPU6050 initialized");
}

void loop() {

  // Collect 64 vibration samples
  for (uint16_t i = 0; i < samples; i++) {

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6, true);

    int16_t rawX = Wire.read() << 8 | Wire.read();
    int16_t rawY = Wire.read() << 8 | Wire.read();
    int16_t rawZ = Wire.read() << 8 | Wire.read();

    float ax = rawX / 16384.0;
    float ay = rawY / 16384.0;
    float az = rawZ / 16384.0;

    // Calculate vibration level
    float vibration = sqrt(
      ax * ax +
      ay * ay +
      (az - 1.0) * (az - 1.0)
    );

    // Store vibration sample for FFT
    vReal[i] = vibration;
    vImag[i] = 0;

    delay(10);
  }

  // Perform FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Find dominant frequency
  double peakFrequency = FFT.majorPeak();

  Serial.println();
  Serial.println("----- FFT ANALYSIS -----");

  Serial.print("Dominant Frequency: ");
  Serial.print(peakFrequency, 2);
  Serial.println(" Hz");

  // Check vibration level
  double maxVibration = 0;

  for (uint16_t i = 0; i < samples; i++) {
    if (vReal[i] > maxVibration) {
      maxVibration = vReal[i];
    }
  }

  Serial.print("Maximum Vibration: ");
  Serial.println(maxVibration, 3);

  if (maxVibration > 0.15) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("WARNING: Abnormal vibration detected!");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("Status: Normal");
  }

  delay(500);
}