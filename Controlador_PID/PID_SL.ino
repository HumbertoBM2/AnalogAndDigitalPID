#include <Wire.h>
#include <Servo.h>

// Definición de objetos y variables
Servo myservo;  // Objeto para controlar el servo
#define AS5600_ADDRESS 0x36  // Dirección I2C del sensor AS5600

// Variables para la medición de ángulos y vueltas
uint16_t anguloAnterior = 0;
int vueltas = 0;
const int motor1Pin1 = 9; // IN1 del motor 1
const int motor1Pin2 = 8; // IN2 del motor 1
const int ENA = 10;

// Variables para el control PID
double Kp = -0.6394, Ki = -1.51, Kd = -0.03217;
const int Mitad = 90;  // Posición central del servo
double Setpoint = 90;  // Setpoint inicial en grados
double Setpoint2 = 0;  // Setpoint ajustado por el sensor
double elapsedTime = 0;
double error = 0, lastError = 0, lastlastError = 0, lastOutput = 0;
double Input, Output = 0, Output2;

// Variables para el control de tiempo
unsigned long currentTime, previousTime = 0;

// Variables para el promedio de lecturas de A0
const int numLecturas = 20;
int lecturasA0[numLecturas];
int indiceLectura = 0;
long sumaLecturas = 0;
bool bufferLleno = false;

void adelante(unsigned int velocidadPWM) {
  // Mueve el motor hacia adelante con velocidad PWM
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(ENA, velocidadPWM);  // Controlar la velocidad del motor usando PWM
}

void atras(unsigned int velocidadPWM) {
  // Mueve el motor hacia atrás con velocidad PWM
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(ENA, velocidadPWM);  // Controlar la velocidad del motor usando PWM
}

void stop() {
  // Detener el motor
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(ENA, 0);  // Desactivar la señal PWM
}

void setup() {
  Serial.begin(115200);  // Inicia la comunicación serial
  Wire.begin();  // Inicia la comunicación I2C
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(ENA, OUTPUT);
  // Inicializa el array de lecturas con ceros
  for (int i = 0; i < numLecturas; i++) {
    lecturasA0[i] = 0;
  }
}

void loop() {
  uint16_t anguloActual = leerAngulo();  // Lee el ángulo actual del sensor
  detectarVueltas(anguloActual);  // Detecta y cuenta las vueltas completas
  float anguloAct = map(anguloActual, 0, 4077, 0, 360);  // Convierte a grados
  float anguloTotal = (vueltas * 360) + anguloAct;  // Ángulo total en grados considerando vueltas

  // Actualiza el buffer de lecturas de A0 para promediar
  sumaLecturas -= lecturasA0[indiceLectura];  // Resta la lectura más antigua
  lecturasA0[indiceLectura] = analogRead(A0);  // Lee un nuevo valor
  sumaLecturas += lecturasA0[indiceLectura];  // Suma la nueva lectura
  indiceLectura = (indiceLectura + 1) % numLecturas;  // Índice circular

  // Verifica si el buffer está lleno para calcular el promedio
  if (indiceLectura == 0) bufferLleno = true;

  // Calcula el Setpoint promediado
  if (bufferLleno) {
    Setpoint = (float)sumaLecturas / numLecturas;
  } else {
    Setpoint = (float)sumaLecturas / (indiceLectura + 1);
  }
  Setpoint2 = map(Setpoint, 0, 1023, 0, 360);  // Escala el Setpoint a grados

  // Cálculo del PID
  currentTime = millis();  // Tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // Tiempo transcurrido
  lastlastError = lastError;
  lastError = error;
  error = Setpoint2 - anguloTotal;  // Calcula el error entre el Setpoint y el ángulo
  lastOutput = Output;
  Output = lastOutput + (Kd / 0.012) * lastlastError + (-Kp - 2.0 * (Kd / 0.012) + (Ki * 0.012)) * lastError + (Kp + (Kd / 0.012)) * error;  // Ecuación PID
  Output2 = Output + Mitad;  // Ajusta el valor final para el servo

  // Limita el valor de Output2 entre -255 y 255 (PWM)
  if (Output2 < -255) Output2 = -255;
  if (Output2 > 255) Output2 = 255;

  // Control de motor con puente H basado en Output2
  if (Output2 > 0) {
    adelante(abs(Output2));  // Mueve el motor hacia adelante con velocidad PWM
  } else {
    atras(abs(Output2));     // Mueve el motor hacia atrás con velocidad PWM
  }
  delay(9);  // Pausa de 9 ms para suavizar el movimiento

  // Envía datos por serial
  Serial.print(Setpoint2);
  Serial.print(", ");
  Serial.print(anguloTotal);
  Serial.print(", ");
  Serial.println(elapsedTime);

  previousTime = currentTime;  // Actualiza el tiempo anterior
  anguloAnterior = anguloActual;  // Actualiza el ángulo anterior
}

// Función para leer el ángulo del sensor AS5600
uint16_t leerAngulo() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0E);  // Dirección del registro del ángulo
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDRESS, 2);  // Solicita 2 bytes del sensor
  uint16_t angulo = 0;

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();  // Lee byte más significativo
    uint8_t lsb = Wire.read();  // Lee byte menos significativo
    angulo = ((msb << 8) | lsb) & 0x0FFF;  // Combina los bytes y aplica máscara de 12 bits
  }
  return angulo;
}

// Función para detectar y contar vueltas completas
void detectarVueltas(uint16_t anguloActual) {
  int16_t diferencia = anguloActual - anguloAnterior;

  // Detecta vueltas completas hacia atrás o adelante basadas en un umbral
  if (diferencia > 2048) {
    vueltas--;  // Vuelta hacia atrás
  } else if (diferencia < -2048) {
    vueltas++;  // Vuelta hacia adelante
  }
}