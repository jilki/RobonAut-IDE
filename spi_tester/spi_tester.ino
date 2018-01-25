/*
 SCP1000 Barometric Pressure Sensor Display

 Shows the output of a Barometric Pressure Sensor on a
 Uses the SPI library. For details on the sensor, see:
 http://www.sparkfun.com/commerce/product_info.php?products_id=8161
 http://www.vti.fi/en/support/obsolete_products/pressure_sensors/

 This sketch adapted from Nathan Seidle's SCP1000 example for PIC:
 http://www.sparkfun.com/datasheets/Sensors/SCP1000-Testing.zip

 Circuit:
 SCP1000 sensor attached to pins 6, 7, 10 - 13:
 DRDY: pin 6
 CSB: pin 7
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13

 created 31 July 2010
 modified 14 August 2010
 by Tom Igoe
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include <elapsedMillis.h>
//#include <system_stm32f4xx.c>
//#include <stm32f4xx_hal.h>
//#include <stm32f4xx_hal_tim.h>
//#include <timer.h>
//#include <HardwareTimer.h>
//#include "stm32yyxx_hal.c"
#define _servopin  32
#define _motorpin  6
#define magnetic_encoder_cha 33 // PC8
#define magnetic_encoder_chb 41 // PB1
Servo servoMotor;
Servo motorM;

TIM_Encoder_InitTypeDef encoder;
TIM_HandleTypeDef timer;

float ratioP=30;   // 15 jó egyenesre
float ratioD=100;   // 100
float kuszob=0.5;
int baseservo=1550;
int basemotor=1500;
int upper=1610;
float linePosold=0;
float oldPos = 0;

elapsedMillis timeElapsed;

//Sensor's memory register addresses:
const int PRESSURE = 0x1F;      //3 most significant bits of pressure
const int PRESSURE_LSB = 0x20;  //16 least significant bits of pressure
const int TEMPERATURE = 0x21;   //16 bit temperature reading
const byte READ = 0b11111100;     // SCP1000's read command
const byte WRITE = 0b00000010;   // SCP1000's write command
const int delay_us = 15;

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int dataReadyPin = 6;
const int chipSelectPin = 10;

void encoderSetup() {
  pinMode(magnetic_encoder_cha, INPUT);
  pinMode(magnetic_encoder_chb, INPUT);
  
 __TIM2_CLK_ENABLE();
 timer.Instance = TIM3;
 timer.Init.Period = 0xFFFF;
 timer.Init.CounterMode = TIM_COUNTERMODE_UP;
 timer.Init.Prescaler = 0;
 timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 
 encoder.EncoderMode = TIM_ENCODERMODE_TI12;
 
 encoder.IC1Filter = 0x0F;
 encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
 encoder.IC1Prescaler = TIM_ICPSC_DIV4;
 encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
 
 encoder.IC2Filter = 0x0F;
 encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
 encoder.IC2Prescaler = TIM_ICPSC_DIV4;
 encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;

 HAL_TIM_Encoder_Init(&timer, &encoder);
 
 HAL_TIM_Encoder_Start_IT(&timer,TIM_CHANNEL_1);
}

void setup() {
  Serial.begin(115200);
  Serial.print("started\n");

  // start the SPI library:
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  // initalize the  data ready and chip select pins:
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);

  
  pinMode(8,  OUTPUT);

  pinMode(10, OUTPUT);    
  pinMode(51, OUTPUT);
  pinMode(5, OUTPUT);

  
  pinMode(4,  OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2,  OUTPUT);

  pinMode(29,  OUTPUT);
  pinMode(51,  OUTPUT);
  pinMode(31,  OUTPUT);
  pinMode(45,  OUTPUT);

  
  pinMode(44, OUTPUT);


//  pinMode(45, OUTPUT);
//  digitalWrite(45, LOW);
//  pinMode(29, OUTPUT);
//  digitalWrite(29, LOW);


   servoMotor.attach(_servopin);
   servoMotor.writeMicroseconds(baseservo);
   motorM.attach(_motorpin);
   motorM.writeMicroseconds(basemotor);

  encoderSetup();
  // give the sensor time to set up:
  delay(4000);
  Serial.println("Loop");
}

void loop() {
      
  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);

  byte pos[6][8] = {{0}};
  byte ans[6][8] = {{0}};

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

  byte muliDimAns[3][6][8];
  byte tresholdedValues[6]={0};
  for(int j = 0; j < 3; j++){
    tresholdedValues[0] = readTresholdedSensor(29);
    tresholdedValues[1] = readTresholdedSensor(51);
    tresholdedValues[2] = readTresholdedSensor(31);
    tresholdedValues[3] = readTresholdedSensor(45);
    tresholdedValues[4] = readTresholdedSensor(3);
    tresholdedValues[5] = readTresholdedSensor(2);
    //readSensor(5,  pos[j], muliDimAns[j][3]);
    //readSensor(4,  pos[j], muliDimAns[j][4]);
    //readSensor(3,  pos[j], muliDimAns[j][5]);
  }

  printFrontSensorValue(tresholdedValues);
  
  for(int i = 0; i<6; i++){
    for(int j=0; j<8; j++){
      if(muliDimAns[0][i][j] == muliDimAns[1][i][j]){
        ans[i][j] = muliDimAns[0][i][j];
      }else if(muliDimAns[1][i][j] == muliDimAns[2][i][j]){
        ans[i][j] = muliDimAns[1][i][j];
      }else if(muliDimAns[0][i][j] == muliDimAns[2][i][j]){
        ans[i][j] = muliDimAns[0][i][j];       
      }else{
        ans[i][j] = muliDimAns[0][i][j]*0.33 + muliDimAns[1][i][j]*0.33 + muliDimAns[2][i][j]*0.33;        
      }
    }
  }

/*  printSensorValue(8,   muliDimAns[0][0], pos[0]);
  printSensorValue(10,  muliDimAns[0][1], pos[1]);
  printSensorValue(51,  muliDimAns[0][2], pos[2]);
  printSensorValue(5,   muliDimAns[0][3], pos[3]);
  printSensorValue(4,   muliDimAns[0][4], pos[4]);
  printSensorValue(3,   muliDimAns[0][5], pos[5]);
*/
  int num = 0;
  int denum = 0;
  
  /*for(int i = 0; i < 4; i++){
    for(int j = 0; j < 8; j++){
      if(ans[i][j]>160){
        num += ((i*8)+j-16)*ans[i][j];
        denum += ans[i][j];
      }
    }
  }*/

  int linValues[48];
   for(int j = 0; j < 6; j++){
    for(int k = 0; k < 8; k++){
      linValues[j*8+k] = ans[j][k];
      }
    }
   
/*  Serial.println("\nLinearised");
  for(int i=0; i<32; i++){
    Serial.print(linValues[i]);
    Serial.print(" ");
  }*/

//  Serial.println("linearization finished");
  int filtered[44];
 
  for(int i=0; i < 44;i++){
    filtered[i] = linValues[i]*0.11 + linValues[i+1]*0.22 + linValues[i+2]*0.34 + linValues[i+3]*0.22 + linValues[i+4]*0.11;
       if(filtered[i]>70){
        num += (i-22)*filtered[i];
        denum += filtered[i];
      }
  }

/*  Serial.println("\nFiltered");
  for(int i=0; i<44; i++){
    Serial.print(filtered[i]);
    Serial.print(" ");
  }
  
  Serial.print("\n\n\n\nCalcuated line pos: ");
  Serial.println(num/(float)denum+0.5);
  Serial.println(timeElapsed);
*/  
  float linePos=num/(float)denum+0.5;
  
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
  
  int enc_cnt = __HAL_TIM_GET_COUNTER(&timer);
  Serial.print("Counter ");
  Serial.println(enc_cnt);

}

void print_binary(int number, int num_digits) {
    int digit;
    for(digit = num_digits - 1; digit >= 0; digit--) {
        char msg[8];
        sprintf(msg, "%c", number & (1 << digit) ? '1' : '0');
        Serial.print(msg);
    }
}

void printFrontSensorValue(byte data[6]){
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


  //Experience based error correction

/*    ans[2] = magicCorrection(boardNum, 2, 146, ans[2]);
    ans[3] = magicCorrection(boardNum, 3, 146, ans[3]);

    ans[4] = magicCorrection(boardNum, 4, 146, ans[4]);
    ans[5] = magicCorrection(boardNum, 5, 146, ans[5]);
*/
/*    ans[2] = magicCorrection(boardNum, 2, 198, ans[2]);
    if(ans[2]==198)
      ans[2] = ans[1];
    ans[3] = magicCorrection(boardNum, 3, 199, ans[3]);
    if(ans[3]==199)
      ans[3] = ans[4];
    ans[4] = magicCorrection(boardNum, 4, 199, ans[4]);
    ans[5] = magicCorrection(boardNum, 5, 198, ans[5]);
*/   

/*  if(ans[4]==199){
    digitalWrite(boardNum, LOW);
    delayMicroseconds(delay_us);
  
    pos[4] = SPI.transfer(3);
    delayMicroseconds(delay_us);
    ans[4] = SPI.transfer(0xAA);
    digitalWrite(boardNum, HIGH);
    delayMicroseconds(delay_us);
  }

  if(ans[4]==199){
    digitalWrite(boardNum, LOW);
    delayMicroseconds(delay_us);
  
    pos[4] = SPI.transfer(3);
    delayMicroseconds(delay_us);
    ans[4] = SPI.transfer(0xAA);
    digitalWrite(boardNum, HIGH);
    delayMicroseconds(delay_us);
  }

  if(ans[4]==199){
    if(ans[5]!=198)
      ans[4]=(ans[3]+ans[5])/2;
    else
      ans[4]=ans[3];
  }

  if(ans[5]==198 || ans[5]==199){
    digitalWrite(boardNum, LOW);
    delayMicroseconds(delay_us);
  
    pos[5] = SPI.transfer(2);
    delayMicroseconds(delay_us);
    ans[5] = SPI.transfer(0xAA);
    digitalWrite(boardNum, HIGH);
    delayMicroseconds(delay_us);  
  }
  
  if(ans[5]==198 || ans[5]==199){
    digitalWrite(boardNum, LOW);
    delayMicroseconds(delay_us);
  
    pos[5] = SPI.transfer(2);
    delayMicroseconds(delay_us);
    ans[5] = SPI.transfer(0xAA);
    digitalWrite(boardNum, HIGH);
    delayMicroseconds(delay_us);  
  }

  if(ans[5]==198 || ans[5]==199){
    if(ans[4]!=199)
      ans[5]=(ans[4]+ans[6])/2;
    else
      ans[5]=ans[6];
  }

  if(ans[0]==156){
    digitalWrite(boardNum, LOW);
    delayMicroseconds(delay_us);
  
    pos[0] = SPI.transfer(7);
    delayMicroseconds(delay_us);
    ans[0] = SPI.transfer(0xAA);
    digitalWrite(boardNum, HIGH);
    delayMicroseconds(delay_us);  
  }
  */
}

byte magicCorrection(int CS, int pos, byte magicNum, byte currentVal){
  if(currentVal!=magicNum){
      digitalWrite(CS, LOW);
      delayMicroseconds(delay_us);
    
      byte dummyPos = SPI.transfer(pos);
      delayMicroseconds(delay_us);
      byte ans = SPI.transfer(0xAA);
      digitalWrite(CS, HIGH);
      delayMicroseconds(delay_us);
    
    if(ans == magicNum){
      digitalWrite(CS, LOW);
      delayMicroseconds(delay_us);
    
      dummyPos = SPI.transfer(pos);
      delayMicroseconds(delay_us);
      ans = SPI.transfer(0xAA);
      digitalWrite(CS, HIGH);
      delayMicroseconds(delay_us);
    }
      return ans;
  }

 return currentVal;
}

