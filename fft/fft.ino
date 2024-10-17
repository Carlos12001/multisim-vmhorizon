#include "arduinoFFT.h"

/*
  Adjusted code based on the example provided by the creator of the arduinoFFT library
*/

#define SAMPLES 128 // Reduced FFT size to save memory          // FFT size (must be a power of 2)
#define SAMPLING_FREQUENCY 8000 // Sampling frequency adjusted to match common audio standards (8 kHz) // Sampling frequency
#define THRESHOLD 50          // Amplitude threshold
#define TOLERANCE 20          // Frequency tolerance in Hz

/* Create FFT object */
ArduinoFFT<float> FFT = ArduinoFFT<float>();

unsigned int sampling_period_us;
unsigned long microseconds;

float vReal[SAMPLES];
float vImag[SAMPLES];

const int ledPins[3][3] = {
  {2, 3, 4},  // LEDs for the 697 Hz row
  {5, 6, 7},  // LEDs for the 770 Hz row
  {8, 9, 10}  // LEDs for the 852 Hz row
};

// Frequencies to detect
const float lowFreqs[3] = {697, 770, 852};
const float highFreqs[3] = {1209, 1336, 1477};

// Variables to store detections
int lowIndex = -1;
int highIndex = -1;

// Define scale types for PrintVector function
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

void setup() {
  Serial.begin(9600); // Adjusted baud rate for better compatibility
  while (!Serial);
  Serial.println("Ready");
  sampling_period_us = round(1000000.0 / SAMPLING_FREQUENCY);

  // Configure pins as output for the LEDs
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      pinMode(ledPins[i][j], OUTPUT);
      digitalWrite(ledPins[i][j], LOW);
    }
  }
}

void loop() {
  /* Data acquisition */
  for(int i = 0; i < SAMPLES; i++) {
    microseconds = micros();  // Current time
    vReal[i] = analogRead(A0);  // Read analog input
    vImag[i] = 0.0;  // Imaginary part must be zeroed

    // Wait until the next sampling period
    while(micros() - microseconds < sampling_period_us) {}
  }

  /* Print the results of the simulated sampling according to time */
  Serial.println("Data:");
  PrintVector(vReal, SAMPLES, SCL_TIME);

  /* Apply window (optional, improves FFT quality) */
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  Serial.println("Weighed data:");
  PrintVector(vReal, SAMPLES, SCL_TIME);

  /* Perform FFT */
  FFT.compute(FFTDirection::Forward);
  Serial.println("Computed Real values:");
  PrintVector(vReal, SAMPLES, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, SAMPLES, SCL_INDEX);

  /* Compute magnitudes */
  FFT.complexToMagnitude();
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (SAMPLES >> 1), SCL_FREQUENCY);

  /* Frequency detection */
  detectFrequencies(vReal);

  /* Run Once */
  while(1);
}

void detectFrequencies(float *vData) {
  // Turn off all LEDs before updating
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      digitalWrite(ledPins[i][j], LOW);
    }
  }

  // Reset detection variables
  lowIndex = -1;
  highIndex = -1;

  // Search for low frequencies
  for(int i = 0; i < 3; i++) {
    int indexLow = frequencyToIndex(lowFreqs[i] - TOLERANCE);
    int indexHigh = frequencyToIndex(lowFreqs[i] + TOLERANCE);

    // Ensure indices are within bounds
    if(indexLow < 0) indexLow = 0;
    if(indexHigh >= (SAMPLES/2)) indexHigh = (SAMPLES/2) - 1;

    for(int k = indexLow; k <= indexHigh; k++) {
      if(vData[k] > THRESHOLD) {
        lowIndex = i;
        break;
      }
    }
    if(lowIndex != -1) break; // If found, exit loop
  }

  // Search for high frequencies
  for(int i = 0; i < 3; i++) {
    int indexLow = frequencyToIndex(highFreqs[i] - TOLERANCE);
    int indexHigh = frequencyToIndex(highFreqs[i] + TOLERANCE);

    // Ensure indices are within bounds
    if(indexLow < 0) indexLow = 0;
    if(indexHigh >= (SAMPLES/2)) indexHigh = (SAMPLES/2) - 1;

    for(int k = indexLow; k <= indexHigh; k++) {
      if(vData[k] > THRESHOLD) {
        highIndex = i;
        break;
      }
    }
    if(highIndex != -1) break; // If found, exit loop
  }

  // If both frequencies are detected, turn on the corresponding LED
  if(lowIndex != -1 && highIndex != -1) {
    digitalWrite(ledPins[lowIndex][highIndex], HIGH);
    // (Optional) Display on Serial Monitor
    Serial.print("Number detected: ");
    Serial.println((lowIndex * 3) + highIndex + 1);
  }
}

int frequencyToIndex(float freq) {
  if(freq < 0) freq = 0;
  if(freq > (SAMPLING_FREQUENCY / 2)) freq = SAMPLING_FREQUENCY / 2;
  double index = (freq * SAMPLES) / SAMPLING_FREQUENCY;
  return round(index);
}

void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType) {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / SAMPLING_FREQUENCY);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);
        break;
    }
    Serial.print(abscissa, 6);
    if(scaleType == SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
