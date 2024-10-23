#include "arduinoFFT.h"

#define SAMPLES 8
#define SAMPLING_FREQUENCY 8000 // Frecuencia de muestreo
#define THRESHOLD 1          // Umbral muy pequeño
#define TOLERANCE 200           // Tolerancia de 200 Hz

ArduinoFFT<float> FFT = ArduinoFFT<float>();

unsigned int sampling_period_us;
unsigned long microseconds;

float vReal[SAMPLES];
float vImag[SAMPLES];

// Frecuencias a detectar
const float freqs[] = {1129, 770, 852, 3722, 1336, 1477};

// LEDs correspondientes a las frecuencias
const int ledPins[] = {3, 4, 5, 6, 7, 8}; // LEDs en pines 3 a 8

void setup() {
  Serial.begin(9600);
  Serial.println("Listo");
  sampling_period_us = round(1000000.0 / SAMPLING_FREQUENCY);

  // Configura los pines como salida para los LEDs
  for(int i = 0; i < 6; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
    
  }

  pinMode(2, OUTPUT);  // LED parpadeante
  pinMode(10, OUTPUT); // Pin 10 apagado
  digitalWrite(2, LOW);
  digitalWrite(10, HIGH); // Asegurar que el pin 10 está apagado
}

void loop() {
  Serial.println("Init Loop" );
  digitalWrite(2, HIGH); // Enciende el LED 2

  /* Adquisición de datos */
  for(int i = 0; i < SAMPLES; i++) {
    microseconds = micros();  // Tiempo actual
    vReal[i] = analogRead(A0);  // Lee la entrada analógica
    Serial.println(vReal[i] );
    vImag[i] = 0.0;  // La parte imaginaria debe ser cero
    while(micros() - microseconds < sampling_period_us) {}
  }

  /* Aplicar ventana (opcional, mejora la calidad de la FFT) */
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  Serial.println("DESPUES DE WINDOW ");

  /* Realizar FFT */
  FFT.compute(FFTDirection::Forward);
  Serial.println("DESPUES DE COMPUTE ");

  /* Calcular magnitudes */
  FFT.complexToMagnitude();
  Serial.println("DESPUES DE COMPUTE MAGNITUD");


  /* Detección de frecuencias */
  detectFrequencies(vReal);
  Serial.println("DESPUES DE DECTECT ");


  delay(500); // Pausa antes de la siguiente FFT
  digitalWrite(2, LOW);  // Apaga el LED 2
  Serial.println("FINAL ");


}

void detectFrequencies(float *vData) {
  // Apagar todos los LEDs antes de actualizar
  for(int i = 0; i < 6; i++) {
    digitalWrite(ledPins[i], LOW);
  }

  for(int i = 0; i < 6; i++) {
    float targetFreq = freqs[i];

    int indexLow = frequencyToIndex(targetFreq - TOLERANCE);
    int indexHigh = frequencyToIndex(targetFreq + TOLERANCE);

    // Asegurar que los índices están dentro de los límites
    if(indexLow < 0) indexLow = 0;
    if(indexHigh >= (SAMPLES/2)) indexHigh = (SAMPLES/2) - 1;

    for(int k = indexLow; k <= indexHigh; k++) {
      if(vData[k] > THRESHOLD) {
        digitalWrite(ledPins[i], HIGH); // Enciende el LED correspondiente
        Serial.print("Frecuencia detectada: ");
        Serial.print(targetFreq);
        Serial.print(" Hz en LED ");
        Serial.println(ledPins[i]);
        break; // Si se encuentra la frecuencia, no es necesario seguir buscando
      }
    }
  }
}

int frequencyToIndex(float freq) {
  if(freq < 0) freq = 0;
  if(freq > (SAMPLING_FREQUENCY / 2)) freq = SAMPLING_FREQUENCY / 2;
  double index = (freq * SAMPLES) / SAMPLING_FREQUENCY;
  return round(index);
}
