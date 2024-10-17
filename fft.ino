#include <arduinoFFT.h>

#define SAMPLES 1024              // FFT size
#define SAMPLING_FREQUENCY 40000  // Sampling frequency
#define THRESHOLD 50              // Amplitude threshold
#define TOLERANCE 20              // Frequency tolerance in Hz

ArduinoFFT FFT = ArduinoFFT();

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

const int ledPins[3][3] = {
    {2, 3, 4},  // LEDs for the 697 Hz row
    {5, 6, 7},  // LEDs for the 770 Hz row
    {8, 9, 10}  // LEDs for the 852 Hz row
};

// Frequencies to detect
const double lowFreqs[3] = {697, 770, 852};
const double highFreqs[3] = {1209, 1336, 1477};

// Variables to store detections
int lowIndex = -1;
int highIndex = -1;

void setup() {
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  // Configure pins as output for the LEDs
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      pinMode(ledPins[i][j], OUTPUT);
      digitalWrite(ledPins[i][j], LOW);
    }
  }
}

void loop() {
  // 1. Data acquisition
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();    // Current time
    vReal[i] = analogRead(A0);  // Read analog input
    vImag[i] = 0;               // Imaginary part is zero

    // Wait until the next sampling period
    while (micros() - microseconds < sampling_period_us) {
    }
  }

  // 2. Apply window (optional, improves FFT quality)
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

  // 3. Perform FFT
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);

  // 4. Compute magnitude
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  // 5. Frequency detection
  detectFrequencies(vReal);

  // 6. (Optional) Wait a while before the next sampling
  delay(100);
}

void detectFrequencies(double *vData) {
  // Turn off all LEDs before updating
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      digitalWrite(ledPins[i][j], LOW);
    }
  }

  // Reset detection variables
  lowIndex = -1;
  highIndex = -1;

  // Search for low frequencies
  for (int i = 0; i < 3; i++) {
    int indexLow = frequencyToIndex(lowFreqs[i] - TOLERANCE);
    int indexHigh = frequencyToIndex(lowFreqs[i] + TOLERANCE);

    // Ensure indices are within bounds
    if (indexLow < 0) indexLow = 0;
    if (indexHigh >= (SAMPLES / 2)) indexHigh = (SAMPLES / 2) - 1;

    for (int k = indexLow; k <= indexHigh; k++) {
      if (vData[k] > THRESHOLD) {
        lowIndex = i;
        break;
      }
    }
    if (lowIndex != -1) break;  // If found, exit loop
  }

  // Search for high frequencies
  for (int i = 0; i < 3; i++) {
    int indexLow = frequencyToIndex(highFreqs[i] - TOLERANCE);
    int indexHigh = frequencyToIndex(highFreqs[i] + TOLERANCE);

    // Ensure indices are within bounds
    if (indexLow < 0) indexLow = 0;
    if (indexHigh >= (SAMPLES / 2)) indexHigh = (SAMPLES / 2) - 1;

    for (int k = indexLow; k <= indexHigh; k++) {
      if (vData[k] > THRESHOLD) {
        highIndex = i;
        break;
      }
    }
    if (highIndex != -1) break;  // If found, exit loop
  }

  // If both frequencies are detected, turn on the corresponding LED
  if (lowIndex != -1 && highIndex != -1) {
    digitalWrite(ledPins[lowIndex][highIndex], HIGH);
    // (Optional) Display on Serial Monitor
    Serial.print("Number detected: ");
    Serial.println((lowIndex * 3) + highIndex + 1);
  }
}

int frequencyToIndex(double freq) {
  if (freq < 0) freq = 0;
  if (freq > (SAMPLING_FREQUENCY / 2)) freq = SAMPLING_FREQUENCY / 2;
  double index = (freq * SAMPLES) / SAMPLING_FREQUENCY;
  return round(index);
}
