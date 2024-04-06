/* Control PID para los motores (Prueba)*/

#include "definicion_pines.hpp"
#include <PIDControl.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(PB10, PB11); 

double kp = 1;
double ki = 1;
double kd = 0.01;
double Ts = 0.1;

double setPoint1 = 210.0;
double setPoint2 = 210.0;

volatile int encoderPulsosMotor1 = 0;
volatile int encoderPulsosMotor2 = 0;
int nRanuras = 20; // Número de ranuras del encoder
double periodo = 100; // Tiempo para las lecturas del encoder


PIDControl motor1PID(kp, ki, kd, Ts); // Parámetros del PID para el primer motor
PIDControl motor2PID(kp, ki, kd, Ts); // Parámetros del PID para el segundo motor



double obtenerVariableProcesoMotor1();
double obtenerVariableProcesoMotor1();

void setup() {

    attachInterrupt(0, interrupcionEncoderMotor1, RISING); // Lectura de los pulsos
    attachInterrupt(0, interrupcionEncoderMotor2, RISING); // Lectura de los pulsos

    mySerial.begin(9600);
    pinMode(MOTOR_DERECHO, OUTPUT);
    pinMode(MOTOR_IZQUIERDO, OUTPUT);
    digitalWrite(MOTOR_ENABLE, HIGH);    
}

void loop() {
  double processVariable1 = obtenerVariableProcesoMotor1(); // Obtener la variable de proceso del primer motor
  double controlVariable1 = motor1PID.calculate(setPoint1, processVariable1); // Calcular la señal de control para el primer motor

  double processVariable2 = obtenerVariableProcesoMotor2(); // Obtener la variable de proceso del segundo motor
  double controlVariable2 = motor2PID.calculate(setPoint2, processVariable2); // Calcular la señal de control para el segundo motor

  analogWrite(MOTOR_DERECHO, int(controlVariable1));  //Pasamos el parametro de control para los motores 
  analogWrite(MOTOR_IZQUIERDO, int(controlVariable2));

}

void interrupcionEncoderMotor1() {
  encoderPulsosMotor1++;
}

void interrupcionEncoderMotor2() {
  encoderPulsosMotor2++;
}


double obtenerVariableProcesoMotor1() {
  // Calcular la velocidad del motor 1 en RPMs basada en los pulsos del encoder
  double velocidadMotor1 = (encoderPulsosMotor1 * 60000.0) / (periodo * nRanuras);

  return velocidadMotor1;
}

double obtenerVariableProcesoMotor2() {
  // Calcular la velocidad del motor 2 en RPMs basada en los pulsos del encoder
  double velocidadMotor2 = (encoderPulsosMotor2 * 60000.0) / (periodo * nRanuras);
  return velocidadMotor2;
}
