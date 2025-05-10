//-- Sensor ultrassonico
//Incluindo biblioteca Ultrasonic.h
#include "Ultrasonic.h"
#include<Servo.h>
#include "IRremote.h"

#define servopin 3

#define  Echo  13 //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)
#define  Trig 12 //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)

#define ENA 5
#define MotorDrt1 6
#define MotorDrt2 7
#define MotorEsq1 8
#define MotorEsq2 9
#define ENB 10

// Definindo os pinos dos sensores de linha
#define Se A1//esq
#define Sc A2//meio
#define Sd A3//drt

#define Ir_pin A0
#define servopinroll 3
#define servopinpitch 4
#define Echo 13
#define Trig 12
#define EchoAb 11
#define TrigAb 2

int velocidade=0;//alterar com potenciometro
int val1;
int v2=0;
int distancia; 
int distancia2;
int distanciaEsq;
int distanciaDir;
int resultado;

boolean modeobst=false;
boolean modeabsm=false;//true;
boolean modelinha=false;

IRrecv irrecv(Ir_pin); 
decode_results results; 

Ultrasonic ultrasonic(Trig,Echo);
Ultrasonic ultrasonicAb(TrigAb,EchoAb);

int sensoresq;// = digitalRead(Se);
int sensorcent;// = digitalRead(Sc);
int sensordt;// = digitalRead(Sd);

Servo rollservo;
Servo pitchservo;


void setup() {
  //Definições de entrada e saída
  
  rollservo.attach(servopinroll);
  rollservo.write(100);

  pitchservo.attach(servopinpitch);
  pitchservo.write(80);

  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT); 
  
  pinMode(servopin, OUTPUT); 

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
 
  pinMode(MotorDrt1, OUTPUT);
  pinMode(MotorDrt2, OUTPUT);
  pinMode(MotorEsq1, OUTPUT);
  pinMode(MotorEsq2, OUTPUT);


  Serial.begin(9600);// Inicia a comunicação seria com velocidade de 9600 bits por segundo

}

void loop() 
{
  // put your main code here, to run repeatedly:

 //frente


    //acerta a velocidade pelo potenciometro
    velocidade = 200;//map(val1, 0, 1023, 0, 255);    
  
    Serial.print("Velocidade  ");
    Serial.println(velocidade);
    
    delay(2000);

    analogWrite(ENA,velocidade);
    analogWrite(ENB,velocidade);

  //frente
    Serial.println("**************************** frente **********************************************+ ");

    //analogWrite(ENA,velocidade);
    //analogWrite(ENB,velocidade);

    digitalWrite(MotorDrt1, HIGH);
    digitalWrite(MotorDrt2, LOW);
    digitalWrite(MotorEsq1,HIGH);
    digitalWrite(MotorEsq2, LOW);

    delay(1000);
    Serial.println("**************************** tras **********************************************+ ");

    digitalWrite(MotorDrt1, LOW);
    digitalWrite(MotorDrt2, HIGH);
    digitalWrite(MotorEsq1,LOW);
    digitalWrite(MotorEsq2, HIGH);


       delay(1000);

   //direita/tras
   Serial.println("***************************drta/ tras **********************************************+ ");

     digitalWrite(MotorDrt1, LOW);
    digitalWrite(MotorDrt2, HIGH);
    digitalWrite(MotorEsq1,HIGH);
    digitalWrite(MotorEsq2, LOW);

     delay(1000);


//esq/tras
Serial.println("*************************** esq/tras **********************************************+ ");

    digitalWrite(MotorDrt1, HIGH);
    digitalWrite(MotorDrt2, LOW);
    digitalWrite(MotorEsq1,LOW);
    digitalWrite(MotorEsq2, HIGH);

     delay(1000);

 // delay(4000);// Vamos dar um tempo de 4 segundos aqui o suficiente para ligar o carrinho e colocar no chao
}
