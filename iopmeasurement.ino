#include "arduinoFFT.h"
#include <Linear2DRegression.hpp>
#include "QuickMedianLib.h" 
Linear2DRegression *linear2DRegression = new Linear2DRegression();

#define SAMPLES 256 //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 4096*2 //Ts = Based on Nyquist, must be 2 times the highest expected frequency. max 8192 for arduino mega
double val = 0;

unsigned int samplingPeriod;
unsigned long microSeconds;
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values

arduinoFFT FFT = arduinoFFT();

const int ledPin = 3;  // the number of the LED pin

// Variables will change:
int ledState = LOW;  // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 100;  // interval at which to blink (milliseconds)

// const int IRPin = A1;
const int microphonePin = A0;


void setup() {
 
  Serial.begin(9600); //Baud rate for the Serial Monitor
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds
  pinMode(ledPin, OUTPUT);
  Serial.println();
  Serial.print("[");
  pinMode (4, INPUT_PULLUP) ; 
}


double seconds = 0;
int counter = 0;

void loop() {
  

  //play tone for 20 percent of the sampling period then let it settle
  // tone(52, 400, samplingPeriod*0.2);
  // int maximaCounter = 0;
  // int minimaCounter = 0;

  // double maximaTimes[10] = {};
  // double maxima[10] = {};
  // double minimaTimes[10] = {};
  // double minima[10] = {};
  // int prevMaxIndex = -100;
  // int prevMinIndex = -100;
  // double samplingPeriod = 1/SAMPLING_FREQUENCY;
  // Serial.print(samplingPeriod);
  // Serial.print(" gwfwf");

  //long sum = 0;

  // /*Sample SAMPLES times*/
  // for(int i=0; i<SAMPLES; i++)
  // {

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW && counter > 256) {
        ledState = HIGH;
        counter = 0;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    }
  microSeconds = micros(); //Returns the number of microseconds since the Arduino board began running the current script.
  
  
  if (counter < 256) {
    vReal[counter] = analogRead(A4); //Readsx the value from analog pin 0 (A0), quantize it and save it as a real term. (microphone?)
    counter++;
  } else if (counter == 256) {
    Serial.print("[");
    for (uint16_t i = 0; i < SAMPLES; i++) {
      Serial.print(vReal[i]);
      if (i != SAMPLES - 1) {
        Serial.print(", ");
      }
  }
  
  Serial.print("];");
  Serial.println("");
  }


  // if ((i > 0 && i < SAMPLES-1)) {

  //   //if there is a max at index i-1
  //   if (vReal[i-2] < vReal[i-1] && vReal[i] < vReal[i-1]) {
  //     //if there is a previous maximum that is close in time to a new one found, replace the old one if it is bigger.
  //     if (prevMaxIndex != -1 && ((i-1) - prevMaxIndex)*(samplingPeriod) < 0.0005 && vReal[prevMaxIndex] < vReal[i-1]) {
  //       maxima[maximaCounter-1] = vReal[i-1];
  //       maximaTimes[maximaCounter-1] = ((double)(i-1))*samplingPeriod;
  //       maximaCounter--;
    
  //     } else  if (maximaCounter != 10){  
  //       maxima[maximaCounter] = vReal[i-1];   
  //       maximaTimes[maximaCounter] = ((double)(i-1))*samplingPeriod;
  //     }
  //     maximaCounter++;
  //     prevMaxIndex = i;
  //   }

  //   //if there is a min at index i-1
  //   if (vReal[i-2] > vReal[i-1] && vReal[i] > vReal[i-1]) {
  //     //if there is a previous min that is close in time to a new one found, replace the old one if it is smaller.
  //     if (prevMinIndex != -1 && ((i-1) - prevMinIndex)*(samplingPeriod) < 0.0005 && vReal[prevMinIndex] > vReal[i-1]) {
  //       minima[minimaCounter-1] = vReal[i-1];
  //       minimaTimes[minimaCounter-1] = ((double) i-1)*samplingPeriod; 
  //       minimaCounter--;
  //     } else if (minimaCounter != 10){
  //       minima[minimaCounter] = vReal[i-1];
  //       minimaTimes[minimaCounter] = ((double) i-1)*samplingPeriod; 
        
  //     }
  //     minimaCounter++;
  //     prevMinIndex = i;
  //   }
  // }

  //   /*remaining wait time between samples if necessary*/
  //   while(micros() < (microSeconds + samplingPeriod))
  //   {
  //     if (currentMillis - previousMillis >= interval) {
  //       // save the last time you blinked the LED
  //       previousMillis = currentMillis;

  //       // if the LED is off turn it on and vice-versa:
  //       if (ledState == LOW) {
  //         ledState = HIGH;
  //       } else {
  //         ledState = LOW;
  //       }

  //       // set the LED with the ledState of the variable:
  //       digitalWrite(ledPin, ledState);
  //     }
  //   }
  // }
  
  // double steadyState = sum/SAMPLES;
  // //obtain damping constant
  // double logMaxima[10] = {};
  // double x = 0;
  // double y = 0;
  // double x2 = 0;
  // double xy = 0;

  // double peakSampleCount = 0;
  // for (int i = 0; i < 10; i++) {
  //   if (maxima[i] - steadyState > 0) {
  //     logMaxima[i] = log(maxima[i] - steadyState);
  //     Serial.print(maximaTimes[i]);
  //     Serial.print(" ");
  //     x += logMaxima[i];
  //     y += maximaTimes[i];
  //     x2 += logMaxima[i]*logMaxima[i];
  //     xy += logMaxima[i]*maximaTimes[i];
  //     peakSampleCount++;
  //   }
  // }
  // double dampingCoefficient = -(peakSampleCount*xy-x*y)/(peakSampleCount*x2-x2);

  // String data = "";
  // Serial.print("[");
  // for (uint16_t i = 0; i < SAMPLES; i++) {
  //   Serial.print(vReal[i]);
  //   if (i != SAMPLES - 1) {
  //     Serial.print(", ");
  //   }
  // }
  
  // Serial.print("];");
  // Serial.println(data);
  // for (int i = 0; i < maximaCounter; i++) {
  //   Serial.print(maxima[i]);
  //   Serial.print(" ");
  // }
 
  // Serial.println();
  // Serial.println(steadyState);
  // Serial.println(dampingCoefficient);


  // // /*Perform FFT on samples*/
  // FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  // FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  // FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);


  // // PrintVector(vReal, (SAMPLES >> 1), SAMPLING_FREQUENCY);
  // // while(1);




  // // /*Find peak frequency and print peak*/
  // double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  // Serial.println(peak);

  // // Read in the ADC and convert it to a voltage:
  //   int proximityADC = analogRead(IRPin);
  //   float proximityV = (float)proximityADC * 5.0 / 1023.0;
  //   Serial.println(peak);//Print out the most dominant frequency.
    // Serial.print(" ");
  // Serial.println(proximityV);


//}

//models the sample data in the form f(x) = e^(ax)*sin(bx) where x is the 
// void getDampingConstant() {
//   double times[SAMPLES];
//   for (int i = 0; i < SAMPLES; i++) {

//   }

//   double dampingCoefficient = 0;
//   double period = 1/SAMPLING_FREQUENCY;

//   int counter = 0;
//   double logPeaks[10];
  
//   double xy = 0;
//   double x2 = 0;
//   double x = 0;
//   double y = 0;
//   for (int i = 0; i < SAMPLES - 1; i++) {
//     if (vReal[i] > vReal[i - 1] &&
//         vReal[i] > vReal[i + 1]) {
//         logPeaks[counter++] = log(vReal[i]);
//         x += log(vReal[i]);
//         y += period*i;
//         x2 += log(vReal[i])*log(vReal[i]);
//         xy = log(vReal[i])*period*i;
//     }
//     if (counter == 10) {
//         break;
//     }
    
//   }
//   dampingCoefficient = -(xy-x*y/counter)/(x2-x*x/counter);
  

 }



