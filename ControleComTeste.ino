/*
Calculations (for 20ms): 
  System clock 16 Mhz and Prescalar 256;
  Timer 1 speed = 16Mhz/256 = 62.5 Khz    
  Pulse time = 1/62.5 Khz =  16us  
  Count up to = 500ms / 16us = 1250 (so this is the value the OCR register should have)*/  
  
#include <Wire.h>
#include <VL53L0X.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#define RPWM 6
#define LPWM 5
#define REN 9
#define LEN 8
VL53L0X sensor;


const float pi = 3.14159;
float conv = 2*pi/800;

float offset_bolinha = 137;
float bolinha = 0;
float ang_bolinha = 0;
float ang_roda = 0;
float ang_bolinha_1 = 0;
float ang_roda_1 = 0;

int tempo = 0;
int tempo_1 = 0;
int TEMPO = 1000;

float volt_pwm = 255/24;
//float K[4] = {0.0 , 0.0 , 0.0 , 0.0};
//float L1[4][4] = {{0.0,0.0,0.0,0.0},
//                  {0.0,0.0,0.0,0.0},
//                  {0.0,0.0,0.0,0.0},
//                  {0.0,0.0,0.0,0.0}};
//                  
//float B[4] = {0.0,0.0,0.0,0.0};
//                  
//float L[4][2] = {{0.0,0.0},
//                 {0.0,0.0},
//                 {0.0,0.0},
//                 {0.0,0.0}};


float K[4] = {       21.217 , -0.02416 ,        3.8181   ,   -0.2216}; //Ganho do Controlador

float B[4] = {    0.0013082  ,  0.0028087  ,    0.13063    ,  0.27912}; //trocar

float L1[4][4] = {
  {         0.48552 ,   -0.012822  ,   0.020073 ,  -9.038E-05},
  {  -0.0084408   ,   0.50232 ,  2.9282E-05   ,  0.019609},
  {         -3.3366   ,   0.82648    ,    1.011 ,  -0.0089954},
  {          -0.15895   ,   -2.8012  ,  0.0043811   ,   0.96119},
 };

float L[4][2] = { //trocar
  {         0.52544   ,  0.012822},
  {      0.012822   ,   0.49768},
  {        4.4345   ,   -0.82648},
  {             0.595     ,    2.8012},
};


float X[4] = {0, 0, 0, 0};
float X_1[4] = {0, 0, 0, 0};
float uo = 0;
float uo_1 = 0;

float pwr = 0;
int dir = 0;
 
union u_tag1 { // allow long to be read as 4 seperate bytes
   byte b[4]; // 4 bytes to be received over I2C
   long LePos; // encoder position as 4 byte long
} u1;

void setMotor(int dir, int pwmVal, int rpwm, int lpwm, int ren, int len){
  
  if(dir == 1){
    digitalWrite(ren,HIGH);
    analogWrite(rpwm,pwmVal);
    digitalWrite(len,HIGH);
    analogWrite(lpwm,0);
    
  }
  else if(dir == -1){
    digitalWrite(ren,HIGH);
    analogWrite(rpwm,0);
    digitalWrite(len,HIGH);
    analogWrite(lpwm,pwmVal);
  }
  else{
    analogWrite(rpwm,0);
    digitalWrite(ren,LOW);
    analogWrite(lpwm,0);
    digitalWrite(len,LOW);
  }  
}

void ganho(){
  int i[4];
  char k[4][16];
  String strk[4];

  for(int j=0;j<4;j++){
    i[j]=0.0;
    strk[j]="";
    while(Serial.available()==0){}
    k[j][0]=char(Serial.read());
    while(k[j][i[j]] !=';'){
      while(Serial.available()==0){}
      k[j][i[j]+1] = char(Serial.read());
      i[j]++;
    }
    for(int n=0;n<i[j];n++){
      strk[j] = strk[j] + k[j][n]; 
    }
    K[j] = strk[j].toFloat();
  }

  Serial.print("K:[");
  Serial.print(K[0]);
  Serial.print(",");
  Serial.print(K[1]);
  Serial.print(",");
  Serial.print(K[2]);
  Serial.print(",");
  Serial.print(K[3]);
  Serial.print("]");
}

void observador_L1(){
  int i[4][4];
  char l1[4][4][16];
  String strl1[4][4];

  for(int j=0;j<4;j++){
    for(int m = 0;m<4;m++){
      i[m][j]=0.0;
      strl1[m][j]="";
      while(Serial.available()==0){}
      l1[m][j][0]=char(Serial.read());
      while(l1[m][j][i[m][j]] !=';'){
        while(Serial.available()==0){}
        l1[m][j][i[m][j]+1] = char(Serial.read());
        i[m][j]++;
      }
      for(int n=0;n<i[m][j];n++){
        strl1[m][j] = strl1[m][j] + l1[m][j][n]; 
      }
      L1[m][j] = strl1[m][j].toFloat();
    }
  }
}


void observador_L(){
  int i[4][2];
  char l[4][2][16];
  String strl[4][2];

  for(int j=0;j<2;j++){
    for(int m = 0;m<4;m++){
      i[m][j]=0.0;
      strl[m][j]="";
      while(Serial.available()==0){}
      l[m][j][0]=char(Serial.read());
      while(l[m][j][i[m][j]] !=';'){
        while(Serial.available()==0){}
        l[m][j][i[m][j]+1] = char(Serial.read());
        i[m][j]++;
      }
      for(int n=0;n<i[m][j];n++){
        strl[m][j] = strl[m][j] + l[m][j][n]; 
      }
      L[m][j] = strl[m][j].toFloat();
    }
  }
}


void observador_B(){
  int i[4];
  char b[4][16];
  String strb[4];

  for(int j=0;j<4;j++){
    i[j]=0.0;
    strb[j]="";
    while(Serial.available()==0){}
    b[j][0]=char(Serial.read());
    while(b[j][i[j]] !=';'){
      while(Serial.available()==0){}
      b[j][i[j]+1] = char(Serial.read());
      i[j]++;
    }
    for(int n=0;n<i[j];n++){
      strb[j] = strb[j] + b[j][n]; 
    }
    B[j] = strb[j].toFloat();
  }
}

ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;
  tempo = tempo + 1;
  
  ang_bolinha = ((offset_bolinha-bolinha)*(0.001)/(0.150)); //Trocar para Leitura do sensor bola azul
  ang_roda=u1.LePos*conv;

  X[0]=L1[0][0]*X_1[0]+L1[0][1]*X_1[1]+L1[0][2]*X_1[2]+L1[0][3]*X_1[3]+B[0]*uo_1+L[0][0]*ang_bolinha_1+L[0][1]*ang_roda_1;
  X[1]=L1[1][0]*X_1[0]+L1[1][1]*X_1[1]+L1[1][2]*X_1[2]+L1[1][3]*X_1[3]+B[1]*uo_1+L[1][0]*ang_bolinha_1+L[1][1]*ang_roda_1;
  X[2]=L1[2][0]*X_1[0]+L1[2][1]*X_1[1]+L1[2][2]*X_1[2]+L1[2][3]*X_1[3]+B[2]*uo_1+L[2][0]*ang_bolinha_1+L[2][1]*ang_roda_1;
  X[3]=L1[3][0]*X_1[0]+L1[3][1]*X_1[1]+L1[3][2]*X_1[2]+L1[3][3]*X_1[3]+B[3]*uo_1+L[3][0]*ang_bolinha_1+L[3][1]*ang_roda_1;

  uo = -(X[0]*K[0]+X[1]*K[1]+X[2]*K[2]+X[3]*K[3]);
  
  if ( uo > 24 ) {
    uo = 24;
  }
  if ( uo < -24 ) {
    uo = -24;
  }

  if (bolinha < 30 || bolinha >  300 ) {
   uo =0;
  }
  
  pwr = fabs(uo*volt_pwm);
  if( pwr > 255 ){
      pwr = 255;
    }
  // motor direction
  dir = 1;
  if(uo<0){
  dir = -1;
  }
  
  ang_bolinha_1 = ang_bolinha;
  ang_roda_1 = ang_roda;
  X_1[0] = X[0];
  X_1[1] = X[1];
  X_1[2] = X[2];
  X_1[3] = X[3];
  uo_1 = uo;
    
  setMotor(dir,pwr,RPWM,LPWM,REN,LEN);
}

void setup() {

  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    while (1) {}
  }
  
  sensor.startContinuous(5);
  
  
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(LEN,OUTPUT);
  pinMode(REN,OUTPUT);
  digitalWrite(REN,HIGH);
  digitalWrite(LEN,HIGH);
  
  
}

void loop() {

  if(Serial.available()){
    char lido = char(Serial.read());
    // K - ganho; O - Observador
    if(lido=='K'){
      ganho();
    }else if(lido=='O'){
      observador_L1();
      observador_L();
      observador_B();
      
      Serial.println("L1: ");
      for(int n1=0;n1<4;n1++){
        Serial.print("|");
        for(int n2=0;n2<4;n2++){
          Serial.print(L1[n1][n2]);
          Serial.print("|");
        }
        Serial.println("");
      }
      Serial.println("");
      
      Serial.println("");
      
      Serial.println("L: ");
      for(int n1=0;n1<4;n1++){
        Serial.print("|");
        for(int n2=0;n2<2;n2++){
          Serial.print(L[n1][n2]);
          Serial.print("|");
        }
        Serial.println("");
      }
      Serial.println("");

      Serial.println("");
      
      Serial.println("B: ");
      for(int n1=0;n1<4;n1++){
        Serial.print("|");
        Serial.print(B[n1]);
        Serial.print("|");
      }
      Serial.println("");
    }else if(lido=='T'){

      int i=0;
      char T[15];
      String strT="";
      
      while(Serial.available()==0){}
      T[i]=char(Serial.read());
      while(T[i]!=';'){
        while(Serial.available()==0){}
        T[i+1]=char(Serial.read());
        i++;
      }
      for(int n=0;n<i;n++){
        strT = strT + T[n]; 
      }
      TEMPO = strT.toInt();
      
      //Serial.println("tempo,ang_bolinha,ang_roda,ang_bolinha_O,vel_bolinha_O,ang_roda_O,vel_roda_O,uo");
      Serial.println("ang_bolinha,ang_roda,ang_bolinha_O,vel_bolinha_O,ang_roda_O,vel_roda_O");
      cli();
    
      TCCR1A = 0;
      TCCR1B = 0;
      TCCR1B |= B00000100;
      TIMSK1 |= B00000010;
      OCR1A = 1249;
      tempo = 0;
      tempo_1 = 0;
      sei();
      
    }
  }
  
  Wire.requestFrom(8, 4);    // request 6 bytes from slave device #8
  for (int i=0; i<4; i++) //requestFrom() is a looping code; it terminates when all requested has come :: thanks GolamMostafa!!
  {
      u1.b[i] = Wire.read();  //data bytes come from FIFO Buffer that has been filled up by requestFrom()
  }
  
  bolinha = sensor.readRangeContinuousMillimeters();

  if(tempo!=tempo_1){
      
      Serial.print(tempo*20);
      Serial.print(",");
      Serial.print(ang_bolinha*180/pi);
      Serial.print(",");
      Serial.print(ang_roda*180/pi);
      Serial.print(",");
      Serial.print(X[0]*180/pi);
      Serial.print(",");
      Serial.print(X[1]*180/pi);
      Serial.print(",");
      Serial.print(X[2]*180/pi);
      Serial.print(",");
      Serial.print(X[3]*180/pi);
      Serial.print(",");
       Serial.print(uo);
      Serial.println("");
      tempo_1=tempo;
      if (tempo>=TEMPO-1){
        TIMSK1 = 0;
        setMotor(1,0,RPWM,LPWM,REN,LEN);
      }
  }
}
