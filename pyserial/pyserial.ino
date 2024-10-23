void setup() {
  Serial.begin(9600); // Inicia la comunicación serial a 9600 baudios
  for (int i = 2; i <= 10; i++) {
    pinMode(i, OUTPUT); // Configura los pines 2-10 como salida para los LEDs
  }
}

void loop() {
  int ledState = Serial.parseInt(); // Lee el estado de los LEDs enviado desde Python
  for (int i = 2; i <= 10; i++) {
    if (ledState & (1 << (i - 2))) {
      digitalWrite(i, HIGH); // Enciende el LED
    } else {
      digitalWrite(i, LOW); // Apaga el LED
    }
  }

  delay(100); // Pequeña demora para estabilizar la lectura
}
