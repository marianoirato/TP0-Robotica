#include "definicion_pines.hpp"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(PB10, PB11); 

double periodo = 100; // Tiempo para las lecturas del encoder
volatile unsigned long tiempoAnterior = 0;
static volatile unsigned long lastTimeDebounce = 0; // Tiempo del rebote

double RPMs[2] = {0}; // Arreglo para almacenar las RPMs de los dos motores
double RPMsFilter[2] = {0}; // Arreglo para almacenar las RPMs filtradas de los dos motores
double alpha = 0.08; // Filtro de media exponencial

volatile int frecuencia[2] = {0}; // Arreglo para almacenar el número de pulsos leídos por el Arduino para cada motor
int nRanuras = 20; // Número de ranuras del encoder

unsigned long tiempoAhora = 0;
int pinENCODER[2] = {ENCODER_DERECHO, ENCODER_IZQUIERDO}; // Pines PWM para los dos motores
int pinMOTORES[2] = {MOTOR_DERECHO, MOTOR_IZQUIERDO}; // Pines PWM para los dos motores

double kp = 1;
double ki = 1;
double kd = 0.01;

double SetP[2] = {150.0, 150.0}; // Set Point para los dos motores

double error[2] = {0}; // Arreglo para almacenar los errores de los dos motores
double errorAnt[2] = {0}; // Arreglo para almacenar los errores anteriores de los dos motores
double errorAntAnt[2] = {0}; // Arreglo para almacenar los errores anteriores de los dos motores

double control_valu[2] = {0}; // Arreglo para almacenar las señales de control de los dos motores
double control_valuAnt[2] = {0}; // Arreglo para almacenar las señales de control anteriores de los dos motores
double Ts = 0.1;


void setup() {
  mySerial.begin(9600);
  attachInterrupt(0, readingEncoder, RISING); // Lectura de los pulsos
  pinMode(pinMOTORES[0], OUTPUT);
  pinMode(pinMOTORES[1], OUTPUT);
  pinMode(pinENCODER[0], OUTPUT);
  pinMode(pinENCODER[1], OUTPUT);  
  digitalWrite(ENCODER_ENABLE, HIGH);
  digitalWrite(MOTOR_ENABLE, HIGH);

  tiempoAnterior = 0;
}

void loop() {
  if (millis() - tiempoAnterior >= periodo) {
    for (int i = 0; i < 2; i++) {
      RPMs[i] = (frecuencia[i] * 60000.0) / ((millis() - tiempoAnterior) * nRanuras);
      RPMsFilter[i] = alpha * RPMs[i] + (1.0 - alpha) * RPMsFilter[i]; // Filtro de media exponencial
      frecuencia[i] = 0;

      control_valu[i] = funcionPID(i);
      analogWrite(pinMOTORES[i], int(control_valu[i]));

      mySerial.print(control_valu[i]);     // Variable de control CV
      mySerial.print(",");
      mySerial.print(SetP[i]);   // Set Point SP
      mySerial.print(",");
      mySerial.println(RPMsFilter[i]); // Variable de proceso PV
    }

    tiempoAnterior = millis();
  }

}

void readingEncoder() {
  for (int i = 0; i < 2; i++) {
    if (digitalRead(pinENCODER[i]) && (micros() - lastTimeDebounce >= 500)) {
      lastTimeDebounce = micros();
      frecuencia[i]++;
    }
  }
}

double funcionPID(int index) {
  error[index] = SetP[index] - RPMsFilter[index];
  control_valu[index] = control_valuAnt[index] + (kp + (kd / Ts)) * error[index] + (-kp + ki * Ts - (2 * kd / Ts)) * errorAnt[index] + (kd / Ts) * errorAntAnt[index];
  control_valuAnt[index] = control_valu[index];
  errorAntAnt[index] = errorAnt[index];
  errorAnt[index] = error[index];

  if (control_valu[index] >= 255) {
    control_valu[index] = 200;
  }
  if (control_valu[index] < 0) {
    control_valu[index] = 0;
  }
  return control_valu[index];
}

