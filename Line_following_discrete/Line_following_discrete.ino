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
float linePosFront=0;
float linePosBack=0;

uint8_t frontSensorPos[48]={0};
int denumFront=0;
int numFront=0;
int denumBack=0;
int numBack=0;

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
   Serial.println(' ');
  Serial.print("Back Sensors: ");
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

byte readTresholdedSensor(int boardNum){
  
  digitalWrite(boardNum, LOW);
  delayMicroseconds(delay_us);
  byte returnValue = SPI.transfer(0xAA);
  delayMicroseconds(delay_us);  //////////////////////////////
  digitalWrite(boardNum, HIGH);
  return returnValue;
}

void setup() {
  Serial.begin(115200);
  Serial.print("started\n");

  // start the SPI library:
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  pinMode(8,  OUTPUT); // Front Chipselects
  pinMode(10, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4,  OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2,  OUTPUT);


  pinMode(45,  OUTPUT); //Back chip selects
  pinMode(31,  OUTPUT);
  pinMode(51,  OUTPUT);
  pinMode(29,  OUTPUT);
  
  pinMode(44, OUTPUT);  //Interrupt


  servoMotor.attach(_servopin);
  servoMotor.writeMicroseconds(baseservo);
  motorM.attach(_motorpin);
  motorM.writeMicroseconds(basemotor);

  // give the motor time to set up:
  delay(4000);
  Serial.println("Loop");
}

void loop() {

  //Send interrupt
  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);

  byte tresholdedValues[10]={0};  
    
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


//  printFrontSensorValue(tresholdedValues);
// Serial.println("");
  denumFront=0;
  numFront=0;
  
  for(int i=0; i<6; i++){
    for(int j=0; j<8; j++){
      frontSensorPos[i*8+j]=((tresholdedValues[i])<<j);
      frontSensorPos[i*8+j]=frontSensorPos[i*8+j]>>7;
      if(frontSensorPos[i*8+j]==0){
        denumFront++;
        numFront=numFront+i*8+j;
      }
    }
  }

  denumBack=0;
  numBack=0;
  
  uint8_t backSensorPos[32]={0};
  for(int i=6; i<10; i++){
    for(int j=0; j<8; j++){
      backSensorPos[(i-6)*8+j]=((tresholdedValues[i])<<j);
      backSensorPos[(i-6)*8+j]=backSensorPos[(i-6)*8+j]>>7;
      if(backSensorPos[(i-6)*8+j]==0){
        denumBack++;
        numBack=numBack+(i-6)*8+j;
      }
    }
  }
/* for(int i=0; i<48; i++){
     Serial.print(frontSensorPos[i]);
   }
    Serial.println("");
  */
/*
  for(int i=0; i<32; i++){
     Serial.print(backSensorPos[i]);
  }
  Serial.println("");
*/
  Serial.println(timeElapsed);

  linePosFront=(float)numFront/denumFront;
  linePosBack=(float)numBack/denumBack;
  
  Serial.println(linePosFront);
  Serial.println(linePosBack);
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


