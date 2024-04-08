/*
this code was developed to the 10-84 eduromaa

Grupo 7

Irato       80001
Gonzalez    72853
Juarez      79111
Cotos       82611


*************************************************WARNING*******************************************************
STARTUP: Colocar robot sobre el centro de la linea negra y presionar el boton central PB14 por algunos segundos
luego colocar el robot sobre un tramo recto y dejarlo libre. 
***************************************************************************************************************
*/





#include "definicion_pines.hpp"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(PB10, PB11); 

// Velocidades en PWM
#define MAXIMA_VELOCIDAD_DIRECTA 500
#define VELOCIDAD_INTERMEDIA_DIRECTA 1900
#define MOTOR_PARADO 2047
#define MAXIMA_VELOCIDAD_INVERSA 4095
#define VELOCIDAD_INTERMEDIA_INVERSA 3047

float UMBRAL = 500; // Mayor a este umbral es NEGRO
int SL_IZQUIERDO_VALOR_ANTERIOR = 0;
int SL_DERECHO_VALOR_ANTERIOR = 0;
long tiempo; // timepo que demora en llegar el eco
long distancia; // distancia en centimetros

void calibrar_negro();
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
    digitalWrite(ENCODER_ENABLE, HIGH);
    digitalWrite(SL_ENABLE, HIGH);


    pinMode(LED_Izquierdo, OUTPUT);
    pinMode(LED_Derecho, OUTPUT);
    analogWriteResolution(12); 
    mySerial.begin(9600);

    pinMode(BOTON, INPUT);
    
    pinMode(SONAR_TRIGGER, OUTPUT);          // pin como salida
    pinMode(SONAR_ECHO, INPUT);              // pin como entrada
    pinMode(SONAR_ENABLE, OUTPUT);
    digitalWrite(SONAR_TRIGGER, LOW);        // Inicializamos el pin con 0
    digitalWrite(SONAR_ENABLE, HIGH);
    
}

void loop() {
  if(!digitalRead(BOTON)){
    calibrar_negro();
  }
  
  int SL_IZQUIERDO_VAL = analogRead(SL_IZQUIERDO);
  int SL_DERECHO_VAL = analogRead(SL_DERECHO);


  sonar();

  

  if(distancia < 50){
    parado();
    do{
      
       sonar();
      
      }
     
      while(distancia < 50);

    
  }

  if(SL_IZQUIERDO_VAL > UMBRAL && SL_DERECHO_VAL > UMBRAL){
    digitalWrite(LED_Derecho, HIGH);
    digitalWrite(LED_Izquierdo, HIGH);
    adelante();

    SL_IZQUIERDO_VALOR_ANTERIOR = SL_IZQUIERDO_VAL;
    SL_DERECHO_VALOR_ANTERIOR = SL_DERECHO_VAL;
  }else if(SL_IZQUIERDO_VAL > UMBRAL && SL_DERECHO_VAL < UMBRAL){
    digitalWrite(LED_Izquierdo, HIGH); 
    digitalWrite(LED_Derecho, LOW);
    giro_izquierda();

    SL_IZQUIERDO_VALOR_ANTERIOR = SL_IZQUIERDO_VAL;
    SL_DERECHO_VALOR_ANTERIOR = SL_DERECHO_VAL;
  }else if(SL_IZQUIERDO_VAL < UMBRAL && SL_DERECHO_VAL > UMBRAL){
    digitalWrite(LED_Derecho, HIGH);
    digitalWrite(LED_Izquierdo, LOW);
    giro_derecha();

    SL_IZQUIERDO_VALOR_ANTERIOR = SL_IZQUIERDO_VAL;
    SL_DERECHO_VALOR_ANTERIOR = SL_DERECHO_VAL;
  }else if(SL_IZQUIERDO_VALOR_ANTERIOR > UMBRAL && SL_DERECHO_VALOR_ANTERIOR < UMBRAL){
    digitalWrite(LED_Izquierdo, HIGH); 
    digitalWrite(LED_Derecho, LOW);
    giro_izquierda();    
  }else if(SL_DERECHO_VALOR_ANTERIOR > UMBRAL && SL_IZQUIERDO_VALOR_ANTERIOR < UMBRAL){
    digitalWrite(LED_Derecho, HIGH);
    digitalWrite(LED_Izquierdo, LOW);
    giro_derecha();
  }
  else{
    digitalWrite(LED_Derecho, LOW);
    digitalWrite(LED_Izquierdo, LOW);
    parado();
  }

  mySerial.print("Sens_izq:");
  mySerial.println(distancia);
/*
  mySerial.print("Sens_izq:");

   mySerial.print(analogRead(SL_IZQUIERDO));

  
   mySerial.print(" Sens_der:");

   mySerial.println(analogRead(SL_DERECHO));
*/
  delay(10);
}

void calibrar_negro(){
  int SL_IZQUIERDO_VAL = analogRead(SL_IZQUIERDO);
  int SL_DERECHO_VAL = analogRead(SL_DERECHO);

  UMBRAL = (SL_IZQUIERDO_VAL + SL_DERECHO_VAL) / 2 - 100;
  mySerial.println(UMBRAL);
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


void sonar(){

  
  digitalWrite(SONAR_TRIGGER, HIGH);
  delayMicroseconds(10);               // Enviamos un pulso de 10us
  digitalWrite(SONAR_TRIGGER, LOW);
  
  tiempo = pulseIn(SONAR_ECHO, HIGH);        //  obtenemos el ancho del pulso
  distancia = tiempo/59;  
}
