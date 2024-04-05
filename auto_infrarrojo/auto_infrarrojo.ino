#include "definicion_pines.hpp"
#include <SoftwareSerial.h>

// Velocidades en PWM
#define MAXIMA_VELOCIDAD_DIRECTA 1047
#define VELOCIDAD_INTERMEDIA_DIRECTA 1747
#define MOTOR_PARADO 2047
#define MAXIMA_VELOCIDAD_INVERSA 2347
#define VELOCIDAD_INTERMEDIA_INVERSA 3047

#define UMBRAL 600 // Mayor a este umbral es NEGRO

void adelante();
void reversa();
void giro_derecha();
void giro_izquierda();
void parado();

void setup() {
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(MOTOR_IZQUIERDO, OUTPUT);
    pinMode(MOTOR_DERECHO, OUTPUT);

    pinMode(SL_ENABLE, OUTPUT);
    pinMode(SL_IZQUIERDO, INPUT);
    pinMode(SL_DERECHO, INPUT);

    digitalWrite(MOTOR_ENABLE, HIGH);
    digitalWrite(SL_ENABLE, HIGH);

    pinMode(PA9, OUTPUT);
    pinMode(PB12, OUTPUT);
    analogWriteResolution(12); 
}

void loop() {
  int SL_IZQUIERDO_VAL = analogRead(SL_IZQUIERDO);
  int SL_DERECHO_VAL = analogRead(SL_DERECHO);

  if(SL_IZQUIERDO_VAL > UMBRAL && SL_DERECHO_VAL > UMBRAL){
    adelante();
  }else if(SL_IZQUIERDO_VAL > UMBRAL && SL_DERECHO_VAL < UMBRAL){
    giro_izquierda();
  }else if(SL_IZQUIERDO_VAL < UMBRAL && SL_DERECHO > UMBRAL){
    giro_derecha();
  }else if(SL_IZQUIERDO_VAL < UMBRAL && SL_DERECHO_VAL < UMBRAL){
    reversa();
  }else{
    parado();
  }
}

void parado(){
    analogWrite(MOTOR_IZQUIERDO, MOTOR_PARADO);
    analogWrite(MOTOR_DERECHO, MOTOR_PARADO);  
}

void adelante(){
    analogWrite(MOTOR_IZQUIERDO, MAXIMA_VELOCIDAD_DIRECTA);
    analogWrite(MOTOR_DERECHO, MAXIMA_VELOCIDAD_DIRECTA);
}

void reversa(){
    analogWrite(MOTOR_IZQUIERDO, MAXIMA_VELOCIDAD_INVERSA);
    analogWrite(MOTOR_DERECHO, MAXIMA_VELOCIDAD_INVERSA);
}

void giro_derecha(){
    analogWrite(MOTOR_IZQUIERDO, MAXIMA_VELOCIDAD_DIRECTA);
    analogWrite(MOTOR_DERECHO, MOTOR_PARADO);
}

void giro_izquierda(){
    analogWrite(MOTOR_IZQUIERDO, MOTOR_PARADO);
    analogWrite(MOTOR_DERECHO, MAXIMA_VELOCIDAD_DIRECTA);
}
