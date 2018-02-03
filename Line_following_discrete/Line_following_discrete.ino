// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include <elapsedMillis.h>
#define _servopin  32 //PC9
#define _motorpin  6   
#define magnetic_encoder_cha (uint16_t)1<<6 // PC6  Föld melletti  CHB
#define magnetic_encoder_chb (uint16_t)1<<5 // PB5  5V melletti  CHA  
Servo servoMotor;
Servo motorM;

TIM_Encoder_InitTypeDef encoder;
TIM_HandleTypeDef timer;

int enc_cnt=0;
double tav=0;
double tav2=0;
double velo=0;
double szorzo=0;
int prevTime;
double prevTav = 0;

int baseservo=1500;
int basemotor=1500;
int upper=1610;

float linePosFront=0;
float linePosBack=0;
float orient=0;
float Lsensor=0.208;

uint8_t frontSensorPos[48]={0};
int denumFront=0;
int numFront=0;
int denumBack=0;
int numBack=0;

double kszi=sqrt(2)/2;
double d5=0;
double t5=1;
double Ttime=kszi/3;
double Lvesszo=0.404;
double s1s2=0;
double s1pluss2=0;
double kp=0;
double kd=0;
double uszog=0;
int pwmbe=1500;
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

void encoderSetup() {
  
  
  __TIM3_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  
  GPIO_InitTypeDef sInitEncoderPin1;
  sInitEncoderPin1.Pin                    = magnetic_encoder_cha;  // A GPIO_PIN_6
  sInitEncoderPin1.Mode                   = GPIO_MODE_AF_PP;
  sInitEncoderPin1.Pull                   = GPIO_PULLUP;
  sInitEncoderPin1.Speed                  = GPIO_SPEED_HIGH;
  sInitEncoderPin1.Alternate              = GPIO_AF2_TIM3; // GPIO_AF2_TIM3

  GPIO_InitTypeDef sInitEncoderPin2;
  sInitEncoderPin2.Pin                    = magnetic_encoder_chb; // A GPIO_PIN_7
  sInitEncoderPin2.Mode                   = GPIO_MODE_AF_PP;
  sInitEncoderPin2.Pull                   = GPIO_PULLUP;
  sInitEncoderPin2.Speed                  = GPIO_SPEED_HIGH;
  sInitEncoderPin2.Alternate              = GPIO_AF2_TIM3; // GPIO_AF2_TIM3

  
  HAL_GPIO_Init(GPIOC, &sInitEncoderPin1);
  HAL_GPIO_Init(GPIOB, &sInitEncoderPin2);

 timer.Instance = TIM3;
 timer.Init.Period = 0xFFFF;
 timer.Init.CounterMode = TIM_COUNTERMODE_UP;
 timer.Init.Prescaler = 0;
 timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  
 encoder.EncoderMode = TIM_ENCODERMODE_TI12;
 
 encoder.IC1Filter = 0x0F;
 encoder.IC1Polarity = TIM_ICPOLARITY_RISING;//TIM_INPUTCHANNELPOLARITY_RISING
 encoder.IC1Prescaler = TIM_ICPSC_DIV1;
 encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
 
 encoder.IC2Filter = 0x0F;
 encoder.IC2Polarity = TIM_ICPOLARITY_FALLING;//TIM_INPUTCHANNELPOLARITY_FALLING
 encoder.IC2Prescaler = TIM_ICPSC_DIV1;
 encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;

 HAL_TIM_Encoder_Init(&timer, &encoder);
 
 HAL_TIM_Encoder_Start(&timer,TIM_CHANNEL_1);
}

void setup() {
  Serial.begin(115200);
  Serial.print("started\n");
  Serial.println(kszi);
  // start the SPI library:
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  encoderSetup();
  pinMode(8,  OUTPUT); // Front Chipselects
  pinMode(10, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(14,  OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2,  OUTPUT);


  pinMode(45,  OUTPUT); //Back chip selects
  pinMode(31,  OUTPUT);
  pinMode(51,  OUTPUT);
  pinMode(29,  OUTPUT);
  
  pinMode(44, OUTPUT);  //Interrupt


  servoMotor.attach(_servopin);
  servoMotor.writeMicroseconds(baseservo);
  //motorM.attach(_motorpin);
 // motorM.writeMicroseconds(basemotor);

  // give the motor time to set up:
  delay(4000);
  Serial.println("Loop");
  szorzo=(100*PI/(((48*38)/(13*13))*256))/1000;
  __HAL_TIM_SET_COUNTER(&timer,32767);
  Serial.println(__HAL_TIM_GET_COUNTER(&timer));
  prevTime = micros();
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
  tresholdedValues[2] = readTresholdedSensor(14);
  tresholdedValues[3] = readTresholdedSensor(5);
  tresholdedValues[4] = readTresholdedSensor(8);
  tresholdedValues[5] = readTresholdedSensor(10);

  
  tresholdedValues[6] = readTresholdedSensor(29);
  tresholdedValues[7] = readTresholdedSensor(51);
  tresholdedValues[8] = readTresholdedSensor(31);
  tresholdedValues[9] = readTresholdedSensor(45);


// printFrontSensorValue(tresholdedValues);
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

  linePosFront=(((float)numFront/denumFront)-23.5)*0.006;
  linePosBack=(((float)numBack/denumBack)-15.5)*0.006;
  orient=atan((linePosFront-linePosBack)/Lsensor);

  Serial.println(linePosFront, 8);
  Serial.println(linePosBack, 8);
  Serial.print("Orient: ");
  Serial.println(orient*180/PI);

  enc_cnt=__HAL_TIM_GET_COUNTER(&timer);
  __HAL_TIM_SET_COUNTER(&timer,32767);
  enc_cnt=enc_cnt-32767;
  tav=enc_cnt*szorzo;
  tav2=tav2+tav;
  int currTime = micros();
  int diffTime = currTime - prevTime;
  prevTime = currTime;
  velo=tav/diffTime*1000000;

  if(velo==0){
    velo=0.0000000001;
  
  }
  d5=0.5*velo+0.5;
  t5=d5/velo;
  Ttime=(t5*kszi)/3;
  s1s2=(1/(Ttime*Ttime));
  s1pluss2=-2*kszi*(1/Ttime);
  kp=-(Lvesszo/(velo*velo))*s1s2;
  kd=(Lvesszo/velo)*((s1pluss2)-velo*kp);
  uszog=-(-kd*orient-kp*linePosFront)/PI*180;

  Serial.print("Bemeno szög: ");
  Serial.println(uszog);

  if(uszog>27){
    pwmbe=1000;
  }
  if(uszog<-26.876){
    pwmbe=2100;
  }
  if(uszog<=27 && uszog>=19){
    pwmbe=-614282.3652+107000.859*uszog-6935.889922*uszog*uszog+198.807768*uszog*uszog*uszog-2.127440683*uszog*uszog*uszog*uszog;
  }
  if(uszog<19 && uszog >-20.125){
    pwmbe=1530.583548-15.34430827*uszog;
  }
  if(uszog<=-20.125 && uszog>=-26.876){
    pwmbe=718281.2226+123156.6653*uszog+7911.076957*uszog*uszog+225.1145702*uszog*uszog*uszog+2.39507239*uszog*uszog*uszog*uszog;
  }
  Serial.print("pwmbe: ");
  Serial.println(pwmbe);
  servoMotor.writeMicroseconds(pwmbe);
  delay(11);
}


