
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include <elapsedMillis.h>
#define _servopin  32
#define _motorpin  6     
Servo servoMotor;
Servo motorM;

float ratioP=30;   // 15 jó egyenesre
float ratioD=100;   // 100
float kuszob=0.5;
int baseservo=1550;
int basemotor=1500;
int upper=1610;
float linePosold=0;
float oldPos = 0;
int count=0;

byte pos[10][8] = {{0}};
  byte ans[10][8] = {{0}};
byte muliDimAns[3][6][8];

elapsedMillis timeElapsed;

const int delay_us = 15;

// pins used for the connection with the sensor


void printSensorValue(int boardNum, byte value[8],byte pos[8]){
  Serial.print("\n\nModule: ");
  Serial.println(boardNum);

  for(int i = 0; i < 8; i++){
    Serial.print(value[i], DEC);
    Serial.print("\t");
  }


  Serial.print("\nPos: ");
  for(int i = 0; i < 4; i++){
    Serial.print(pos[(i*2)], DEC);
    Serial.print(": ");
    Serial.print(pos[(i*2)+1], DEC);
    Serial.print("\t");
  }

  Serial.print("\n\n");
}

void readSensor(int boardNum, byte *pos, byte *ans){
  
  for(int i = 0; i < 8; i++)
  {
    digitalWrite(boardNum, LOW);
    delayMicroseconds(delay_us);
    pos[i] = SPI.transfer(7-i);
    delayMicroseconds(delay_us);
    ans[i] = SPI.transfer(0xAA);
    digitalWrite(boardNum, HIGH);
    delayMicroseconds(delay_us);
  } 
}

void setup() {
  Serial.begin(115200);
  Serial.print("started\n");

  // start the SPI library:
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  pinMode(45,  OUTPUT); //Back chip selects
  pinMode(31,  OUTPUT);
  pinMode(51,  OUTPUT);
  pinMode(29,  OUTPUT);

  pinMode(8,  OUTPUT); // Front Chipselects
  pinMode(10, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(14,  OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2,  OUTPUT);
  
  pinMode(44, OUTPUT);  //Interrupt

   servoMotor.attach(_servopin);
   servoMotor.writeMicroseconds(baseservo);
   motorM.attach(_motorpin);
   motorM.writeMicroseconds(basemotor);

  // give the sensor time to set up:
  delay(4000);
  Serial.println("Loop");
}

void loop() {
  
      Serial.println("Loop");
  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);
  Serial.println("Loop1");
  
  //readSensor(51, pos[0], ans[0]);
/*
  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);
  */
  //readSensor(10, pos[1], ans[1]);

/*  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);
  */
  //readSensor(5,  pos[2], ans[2]);

/*  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);
  */
  //readSensor(4,  pos[3], ans[3]);

  
 
    readSensor(45,  pos[0], ans[0]);
    readSensor(31, pos[1], ans[1]);
    readSensor(51, pos[2], ans[2]);
    readSensor(29,  pos[3], ans[3]);

    readSensor(8,  pos[4], ans[4]);
    readSensor(10, pos[5], ans[5]);
    readSensor(5, pos[6], ans[6]);
    readSensor(14,  pos[7], ans[7]);
    readSensor(3,  pos[8], ans[8]);
    readSensor(2, pos[9], ans[9]);
    
  Serial.println("loop2");
  printSensorValue(45,   ans[0], pos[0]);
  Serial.println("loop3");
  printSensorValue(31,  ans[1], pos[1]);
  printSensorValue(51,  ans[2], pos[2]);
  printSensorValue(29,   ans[3], pos[3]);
 
  printSensorValue(8,   ans[4], pos[4]);
  printSensorValue(10,  ans[5], pos[5]);
  printSensorValue(5,  ans[6], pos[6]);
  printSensorValue(14,   ans[7], pos[7]);
  printSensorValue(3,   ans[8], pos[8]);
  printSensorValue(2,  ans[9], pos[9]);
  
  /*
  int num = 0;
  int denum = 0;
  */
  /*for(int i = 0; i < 4; i++){
    for(int j = 0; j < 8; j++){
      if(ans[i][j]>160){
        num += ((i*8)+j-16)*ans[i][j];
        denum += ans[i][j];
      }
    }
  }*/
/*
  int linValues[48];
   for(int j = 0; j < 6; j++){
    for(int k = 0; k < 8; k++){
      linValues[j*8+k] = ans[j][k];
      }
    }
  */ 
/*  Serial.println("\nLinearised");
  for(int i=0; i<32; i++){
    Serial.print(linValues[i]);
    Serial.print(" ");
  }*/

//  Serial.println("linearization finished");
  //int filtered[44];
 /*
  for(int i=0; i < 44;i++){
    filtered[i] = linValues[i]*0.11 + linValues[i+1]*0.22 + linValues[i+2]*0.34 + linValues[i+3]*0.22 + linValues[i+4]*0.11;
       if(filtered[i]>70){
        num += (i-22)*filtered[i];
        denum += filtered[i];
      }
  }
  */
/*
  Serial.println("\nFiltered");
  for(int i=0; i<44; i++){
    Serial.print(filtered[i]);
    Serial.print(" ");
  }
  */
  //Serial.println(timeElapsed);
 /* 
  Serial.print("\n\n\n\nCalcuated line pos: ");
  Serial.println(num/(float)denum+0.5);
  Serial.println(timeElapsed);
 
  float linePos=num/(float)denum+0.5;
  */
/*  Serial.println("\n\n");
  Serial.println(linePos);
  Serial.print("deltahiba:");
  Serial.println(linePos-linePosold);
  Serial.print("ratioD:");
  Serial.println(ratioD);
  Serial.print("Lineposold:");
  Serial.println(linePosold);
  Serial.print("P*hiba:");
  Serial.println(ratioP*linePos);
  Serial.print("D*deltahiba");
  Serial.println(ratioD*(linePos-linePosold));
*/
/*
  if(!isnan(linePos))
  {
    float diff = linePos-linePosold;
    if(diff < 2 && diff > -2){
      if(linePos>5 || linePos < -5){
         motorM.writeMicroseconds(upper);
         servoMotor.writeMicroseconds(baseservo+ratioP*linePos+ratioD*(linePos-linePosold));
         //Serial.print("\n\nbeléptem a felételebe");
       }
       else{
         motorM.writeMicroseconds(upper);
         servoMotor.writeMicroseconds(baseservo+ratioP*linePos+ratioD*(linePos-linePosold));
       }
    }
    linePosold=linePos;
  }
  else{
      if(linePosold<-12){
        servoMotor.writeMicroseconds(1000);
      }
      if(linePosold > 12){
        servoMotor.writeMicroseconds(2000);
      }
       //servoMotor.writeMicroseconds(base);
       motorM.writeMicroseconds(basemotor);
      }
*/
      
}

