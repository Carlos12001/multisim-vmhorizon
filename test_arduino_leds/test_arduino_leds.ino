const int ledPins[3][3] = {
  {2, 5, 8},
  {3, 6, 9},
  {4, 7, 10}
};

void setup() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      pinMode(ledPins[i][j], OUTPUT);
    }
  }
}

void loop() {
  for (int i = 0; i <= 9; i++) {
    apagarLeds();
    if (i < 9) {
      int fila = i / 3;
      int columna = i % 3;
      digitalWrite(ledPins[fila][columna], HIGH);
    }
    delay(1000);
  }
}

void apagarLeds() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      digitalWrite(ledPins[i][j], LOW);
    }
  }
}
