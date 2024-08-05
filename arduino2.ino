#include "arduinoFFT.h"

#define SAMPLES 128
#define SAMPLING_FREQUENCY 4096*2 

unsigned int samplingPeriod;
unsigned long microSeconds;
double vReal[SAMPLES];
volatile int sampleCounter = 0; 
volatile bool readFlag = false;
unsigned long previousMillis = 0;
const long interval = 100; 
const int ledPin = 3;  
const int microphonePin = A4;

arduinoFFT FFT = arduinoFFT();

void pin4Interrupt() {
  readFlag = true; 
}

void setup() {
  Serial.begin(9600);
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY));
  pinMode(ledPin, OUTPUT);
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(4), pin4Interrupt, RISING); 
  Serial.println("Ready to read..."); 
}

void loop() {
  unsigned long currentMillis = millis();

  // Blink LED
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    digitalWrite(ledPin, !digitalRead(ledPin));  // Toggle LED
    sampleCounter = 0;
    readFlag = true;
  }

  if (readFlag && sampleCounter < SAMPLES) {
    microSeconds = micros();
    vReal[sampleCounter] = analogRead(microphonePin);
    sampleCounter++;

    // while (micros() - microSeconds < samplingPeriod) {
    //   // Wait for sampling period
    // }
  }

  // Check if all samples are collected
  if (sampleCounter == SAMPLES) {
    readFlag = false; // Stop reading

    // Print results 
    Serial.print("[");
    for (int i = 0; i < SAMPLES; i++) {
      Serial.print(vReal[i]);
      if (i != SAMPLES - 1) {
        Serial.print(", ");
      }
    }
    Serial.println("];");
  }
}
