// the sensor communicates using SPI, so include the library:
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

elapsedMillis timeElapsed;

const int delay_us = 50;

void print_binary(int number, int num_digits) {
    int digit;
    for(digit = num_digits - 1; digit >= 0; digit--) {
        char msg[8];
        sprintf(msg, "%c", number & (1 << digit) ? '1' : '0');
        Serial.print(msg);
    }
}

void printFrontSensorValue(byte data[10]){
  Serial.print("\n\nFront Sensors: ");
  char msg[8];
  //sprintf(msg, "%08b",data[0]);
  print_binary(data[0], 8);
  Serial.print(' ');
  print_binary(data[1], 8);
  Serial.print(' ');
  print_binary(data[2], 8);
  Serial.print(' ');
  print_binary(data[3], 8);
  Serial.print(' ');
  print_binary(data[4], 8);
  Serial.print(' ');
  print_binary(data[5], 8);
   Serial.print(' ');
  print_binary(data[6], 8);
  Serial.print(' ');
  print_binary(data[7], 8);
  Serial.print(' ');
  print_binary(data[8], 8);
  Serial.print(' ');
  print_binary(data[9], 8);
}

void printTresholdedSensorValue(int boardNum, byte data){
  Serial.print("\n\nModule: ");
  Serial.println(boardNum);
  Serial.print("\t");
  Serial.print(data, BIN);
}

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

byte readTresholdedSensor(int boardNum){
  
  digitalWrite(boardNum, LOW);
  delayMicroseconds(delay_us);
  byte returnValue = SPI.transfer(0xAA);
  delayMicroseconds(delay_us);  //////////////////////////////
  digitalWrite(boardNum, HIGH);
  return returnValue;
}

void readSensor(int boardNum, byte *pos, byte *ans){
  
  for(int i = 0; i < 8; i++)
  {
    digitalWrite(boardNum, LOW);
    delayMicroseconds(delay_us);
  
    pos[i] = SPI.transfer(7-i);
    delayMicroseconds(delay_us);
    if(pos[i]!=227){
        pos[i] = SPI.transfer(7-i);
        delayMicroseconds(delay_us);
    }
    ans[i] = SPI.transfer(0xAA);
    digitalWrite(boardNum, HIGH);
    delayMicroseconds(delay_us);
    if(ans[i]==227)
      i--;
  } 
}

void setup() {
  Serial.begin(115200);
  Serial.print("started\n");

  // start the SPI library:
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  pinMode(8,  OUTPUT);

  pinMode(10, OUTPUT);
  pinMode(5, OUTPUT);

  
  pinMode(4,  OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2,  OUTPUT);


  pinMode(45,  OUTPUT); 
  pinMode(31,  OUTPUT);
  pinMode(51,  OUTPUT);
  pinMode(29,  OUTPUT);
  
  pinMode(44, OUTPUT);

   servoMotor.attach(_servopin);
   servoMotor.writeMicroseconds(baseservo);
   motorM.attach(_motorpin);
   motorM.writeMicroseconds(basemotor);

  // give the motor time to set up:
  delay(4000);
  Serial.println("Loop");
}

void loop() {
      
  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);

  byte tresholdedValues[10]={0};
  //for(int j = 0; j < 3; j++){
   
    
    tresholdedValues[0] = readTresholdedSensor(2);
    tresholdedValues[1] = readTresholdedSensor(3);
    tresholdedValues[2] = readTresholdedSensor(4);
    tresholdedValues[3] = readTresholdedSensor(5);
    tresholdedValues[4] = readTresholdedSensor(8);
    tresholdedValues[5] = readTresholdedSensor(10);

    
    tresholdedValues[6] = readTresholdedSensor(29);
    tresholdedValues[7] = readTresholdedSensor(51);
    tresholdedValues[8] = readTresholdedSensor(31);
    tresholdedValues[9] = readTresholdedSensor(45);
    
  //}

  printFrontSensorValue(tresholdedValues);
  



  int num = 0;
  int denum = 0;

  float linePos=num/(float)denum+0.5;
  
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
}


