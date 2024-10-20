#include <Wire.h>

// Definición de pines para el puente H
#define IN1 9  // Pin para controlar la dirección 1
#define IN2 8  // Pin para controlar la dirección 2
#define ENA 10  // Pin para el control de velocidad (PWM)

// Dirección I2C del sensor AS5600
#define AS5600_ADDRESS 0x36  

// Variables para la medición de ángulos y vueltas
uint16_t anguloAnterior = 0;
int vueltas = 0;

// Variables para la generación de valores aleatorios y tiempo
unsigned long seed = 0xFFFF;  // Semilla para el generador de números pseudoaleatorios
unsigned long tiempoAnterior = 0;
unsigned long tiempoActual = 0;

void setup() {
  Serial.begin(115200);  // Inicia la comunicación serial
  Wire.begin();  // Inicia la comunicación I2C
  
  // Configura los pines del puente H como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void loop() {
  // Lee el ángulo actual del sensor y calcula el ángulo total considerando las vueltas completas
  uint16_t anguloActual = leerAngulo();  // Lee el ángulo actual del sensor AS5600
  detectarVueltas(anguloActual);  // Detecta y cuenta las vueltas completas
  float anguloAct = map(anguloActual, 0, 4077, 0, 360.0);  // Convierte el valor a grados
  float anguloTotal = (vueltas * 360) + anguloAct;  // Ángulo total en grados

  // Generación de un número pseudoaleatorio basado en la semilla
  unsigned long xo = seed ^ (seed >> 3) >> 1;
  unsigned long vel = (xo & 0x0001);  // Determina la velocidad aleatoria (0 o 1)
  seed = (seed >> 1);
  seed = (seed + (vel << 31));  // Actualiza la semilla

  // Controla el motor DC según el valor aleatorio de velocidad
  if (vel == 1) {
    moverMotor(255);  // Velocidad máxima en una dirección
  } else {
    moverMotor(-255);  // Velocidad máxima en la dirección opuesta
  }
  
  // Envía los datos por el puerto serial
  Serial.print(anguloTotal);  // Imprime el ángulo total
  Serial.print(", ");
  Serial.print(vel == 1 ? 255 : -255);  // Imprime la velocidad (positiva o negativa)
  Serial.print(", ");

  // Calcula el tiempo transcurrido entre iteraciones y lo imprime
  tiempoAnterior = tiempoActual;
  tiempoActual = millis();
  int dt = tiempoActual - tiempoAnterior;
  Serial.print(dt);  // Imprime el tiempo transcurrido
  Serial.println(";");

  // Actualiza el ángulo anterior para la próxima iteración
  anguloAnterior = anguloActual;
  
  // Pequeña pausa antes de la siguiente lectura
  delay(10);  // Espera 10 ms antes de la siguiente lectura
}

// Función para leer el ángulo del sensor AS5600
uint16_t leerAngulo() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0E);  // Registro del ángulo (MSB)
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDRESS, 2);  // Solicita 2 bytes del sensor
  uint16_t angulo = 0;

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();  // Lee el byte más significativo
    uint8_t lsb = Wire.read();  // Lee el byte menos significativo
    angulo = ((msb << 8) | lsb) & 0x0FFF;  // Combina los bytes y aplica la máscara de 12 bits
  }
  return angulo;
}

// Función para detectar y contar vueltas completas
void detectarVueltas(uint16_t anguloActual) {
  int16_t diferencia = anguloActual - anguloAnterior;

  // Detecta vueltas completas hacia atrás o adelante basadas en un umbral
  if (diferencia > 2000) {
    vueltas--;  // Vuelta hacia atrás
  } else if (diferencia < -2000) {
    vueltas++;  // Vuelta hacia adelante
  }
}

// Función para controlar el motor DC
void moverMotor(int velocidad) {
  if (velocidad > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, velocidad);  // Controla la velocidad del motor con PWM
  } else if (velocidad < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -velocidad);  // Controla la velocidad en la dirección opuesta
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);  // Detiene el motor
  }
}