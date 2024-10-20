int in1 = 8;  // Pin que controla el sentido de giro Motor A
int in2 = 9;  // Pin que controla el sentido de giro Motor A
int EnA = 10; // Pin que controla la velocidad del Motor A

void setup() {
  pinMode(in1, OUTPUT);    // Configura los pines como salida
  pinMode(in2, OUTPUT);
  pinMode(EnA, OUTPUT);
}

void loop() {
  analogWrite(EnA, 100);  // Velocidad del Motor A
  digitalWrite(in1, HIGH); // GIRO DERECHA
  digitalWrite(in2, LOW);
  delay(3000);

  analogWrite(EnA, 150);  // Velocidad del Motor A
  delay(3000);

  analogWrite(EnA, 255); // Velocidad del Motor A
  delay(3000);

    analogWrite(EnA, 100);  // Velocidad del Motor A
  digitalWrite(in2, HIGH); // GIRO DERECHA
  digitalWrite(in1, LOW);
  delay(3000);

  analogWrite(EnA, 150);  // Velocidad del Motor A
  delay(3000);

  analogWrite(EnA, 255); // Velocidad del Motor A
  delay(3000);
}
