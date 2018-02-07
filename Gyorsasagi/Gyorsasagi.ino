// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include <elapsedMillis.h>
#include "String.h"

#define _servopin  32 //PC9
#define _motorpin  7  //PA8
#define magnetic_encoder_cha (uint16_t)1<<6 // PC6  Föld melletti  CHB
#define magnetic_encoder_chb (uint16_t)1<<5 // PB5  5V melletti  CHA  
Servo servoMotor;
Servo motorM;

//Analog labak távolsághoz
//Baloldali: PB0 49
//Jobboldali: PA1 47
//Első: PA0 46

TIM_Encoder_InitTypeDef encoder;
TIM_HandleTypeDef timer;

#define pin_tx (uint16_t)1<<10  //PB10
#define pin_rx (uint16_t)1<<5   //PC5
//PB7-->D22 kezdest jelzo bit
//PB8-->D15 Infra interrupt 
//PB12-->D38 Inerc interrupt 

UART_HandleTypeDef uart3struct;

double uszogOld=0;
float ratioP=100;
float ratioD=30;



int cycleCount = 0;
int lastVonalSzam=1;
//Motor_Controller
int basemotor=1500;

int motorFord=1500;
double Kc=1;//0.6158
double zd=0.9678;

double u1=0;
double u2=0;
double u=0;
double u2past=0;
double upast=0;

double alap=0.7;
const double constVelo=0.7;////////////////////////////////////////////Use this to set velocity in m/s
double beta=65.848;
double alfa=0.0422;

//Velocity meas with encoder
int enc_cnt=0;
double tav=0;
double tav2=0;
double velo=0;
double szorzo=0;
int prevTime;
double prevTav = 0;

double tavFin=0;
double tavFin2=0;

int alapTav=350;
double safetytav=0;
double safetyP=0.004;

float tav_dron_pre=0;
float tav_dron=0;
int i_dron=0;

float tav_harom=0;
float tav_egy=0;

int baseservo=1500;
int upper=1610;

//Line meas
byte tresholdedValues[10]={0};
float linePosFront=0;
float linePosFrontOld=0;
float linePosBack=0;
double orient=0;
double lastOrient=0;
float Lsensor=0.208;

uint8_t frontSensorPos[48]={0};
int denumFront=0;
int numFront=0;
int denumBack=0;
int numBack=0;
int forDifference[48]={0};
int vonalSzam=1;
int prevVonalSzam=1;
double tavok[3]={0};
double temp=0;
int vonalFajta[3]={0};
int counterTav=0;
double tulHosszu=0.2;

//Állapotokhoz
char state=0;
//0: safety car
//1: gyors
//2: lassú
//3: fekezes

int dronWas=0;
int counterDron=0;
int infraDron=0;
int diffInfraDron=0;

int lassuVege = 0;


//State-space controller
double kszi=sqrt(2)/2;
double d5=0;
double t5=1;
double Ttime=kszi/3;
double Lvesszo=0.395;
double Lhossz=0.28;
double s1s2=0;
double s1pluss2=0;
double kp=0;
double kd=0;
double uszog=0;
int pwmbe=1500;
elapsedMillis timeElapsed;

//UART Nucleo es arduino kozott
float ypr0=0;
float ypr1=0;
float infr=0;
float korf=0;
float haromtav = 0;
float egytav=0;

double ido=0; //UART ido szamlaloja


const int delay_us = 50;


void print_binary(int number, int num_digits) {
    int digit;
    for(digit = num_digits - 1; digit >= 0; digit--) {
        char msg[8];
        sprintf(msg, "%c", number & (1 << digit) ? '1' : '0');
        Serial.print(msg);
    }
}

void piControll(){
  /*============================================
  =          PI velocity controller          =
  ============================================*/
   u2=zd*u2past+(1-zd)*upast;
   u1=Kc*(alap-velo);
   u=(u1+u2);
   if(u>6)
    u=6;
   if(u<0)
    u=0;
   
   motorFord=(u+beta)/alfa;
   /*
   Serial.println(motorFord);
   Serial.println(velo);
   */
   motorM.writeMicroseconds(motorFord);
   u2past=u2;
   upast=u; 
}

void lineControll(){
    /*============================================
    =      State-space line following          =
    ============================================*/
    //
    
    t5=d5/velo;
    Ttime=(t5*kszi)/3;
    s1s2=(1/(Ttime*Ttime));
    s1pluss2=-2*kszi*(1/Ttime);
    kp=-(Lvesszo/(velo*velo))*s1s2;
    kd=(Lvesszo/velo)*((s1pluss2)-velo*kp);
    uszog=-(-kd*orient-kp*linePosFront)/PI*180;
    uszog=uszog*Lvesszo/Lhossz;

    /*
    Serial.print("Bemeno szög: ");
    Serial.println(uszog);
    */
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
    /*
    Serial.print("pwmbe: ");
    Serial.println(pwmbe);
    */
    servoMotor.writeMicroseconds(pwmbe);
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

void safetyCar(){
  safetytav = analogRead(46);  //1 m-re 221  0,0075

  alap=safetyP*(alapTav-safetytav)+1.2;  //negatív jön, akkor lassítani, pozitív, akkor gyorsítani
  kszi=0.7;
   d5=1*velo+0.5; //0.75
   if(velo<0.6){
     d5=2;
   }
   piControll();
  lineControll();
  /*
  if(safetytav>410){
    motorM.writeMicroseconds(1000);
  }
  if(safetytav<380){
    alap=1.7;
    piControll();
  }
  */
}

void gyors(){
   kszi=0.7;
   d5=1*velo+0.5; //0.75
   if(velo<0.6){
     d5=2;
   }
   lineControll();

  //PI
   alap=3;
   u2=zd*u2past+(1-zd)*upast;
   u1=Kc*(alap-velo);
   u=(u1+u2);
   if(u>6)
    u=6;
   /*if(u<0)
    u=0;
   */
   motorFord=(u+beta)/alfa;
   /*
   Serial.println(motorFord);
   Serial.println(velo);
   */
   motorM.writeMicroseconds(motorFord);
   u2past=u2;
   upast=u; 

   if(vonalSzam==3){
    haromtav = haromtav+tav;
   }
   if(haromtav>0.5){
    motorM.writeMicroseconds(1000);
    state = 3;
    upast=0;
    u2past=0;
    haromtav=0;
   }
   
}

void fekez(){
  if(velo>0.75){
    motorM.writeMicroseconds(1250);
    lineControll();
  }
  else{
    state=2;
    upast=0;
    u2past=0;
  }
}
void lassu(){
  //PI
   alap=1;
   u2=zd*u2past+(1-zd)*upast;
   u1=Kc*(alap-velo);
   u=(u1+u2);
   if(u>6)
    u=6;
    /*
   if(u<0)
    u=0;
   */
   motorFord=(u+beta)/alfa;
   motorM.writeMicroseconds(motorFord);

   u2past=u2;
   upast=u; 
   /*
   Serial.println(motorFord);
   Serial.println(velo);
   */
     ratioP=200;  ///1000-nél állandó lengés szögben, 800-nél leng, 150-nél még leng egy kicsit
  //ratioD=
  ratioD=1/3;  //Tu=~2  //1/4 volt az eredeti, egyenesre majdnem jó
  //uszog=-(ratioP*linePosFront)/PI*180;
    uszog=(-(ratioP*linePosFront+ratioD*(linePosFront-linePosFrontOld)/0.02));
// uszog=(linePosFront-0.992*linePosFrontOld+327.6 *uszogOld)/330;
  //valtozo=baseservo+ratioP*linePosFront+ratioD*(linePosFront-linePosFrontOld);
  

    if(uszog>27)
    {
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
    Serial.println(pwmbe);
    servoMotor.writeMicroseconds(pwmbe);



   if(vonalSzam==1){
    egytav = egytav+tav;
   }
   if(egytav>0.1){
    lassuVege=1;
   }
   if(vonalSzam==3 && lassuVege){
     haromtav = haromtav+tav;
   }
   
   if(haromtav>.30){
    state = 1;
    upast=0;
    u2past=0;
    egytav=0;
    haromtav = 0;
    lassuVege = 0;
   }
  
}

void setup() {
  Serial.begin(115200);
  Serial.print("started\n");
  //Serial.println(kszi);
  // start the SPI library:
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  encoderSetup();
  pinMode(8,  OUTPUT); // Front Chipselects Threshold: 0x8000
  pinMode(10, OUTPUT);  
  pinMode(5, OUTPUT);
  pinMode(14,  OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2,  OUTPUT);


  pinMode(45,  OUTPUT); //Back chip selects Threshold: 0xA000
  pinMode(31,  OUTPUT);
  pinMode(51,  OUTPUT); //B000
  pinMode(29,  OUTPUT);
  
  pinMode(44, OUTPUT);  //Interrupt

  //UART inic
  pinMode(38 ,OUTPUT);
  pinMode(15,OUTPUT);
  pinMode(22,INPUT);
  digitalWrite(38, HIGH);
  digitalWrite(15, HIGH);
  digitalWrite(22,HIGH);
  delay(1000); ///////////////////////////////////////////////////////////////////////Ez nemtom kell e ide///////////////////
  ido=timeElapsed;

  

  servoMotor.attach(_servopin);
  servoMotor.writeMicroseconds(baseservo);
  motorM.attach(_motorpin);
  motorM.writeMicroseconds(basemotor);

  // give the motor time to set up:
  delay(3000);
  Serial.println("Loop");
  szorzo=(100*PI/(((48*38)/(13*13))*256))/1000;
  __HAL_TIM_SET_COUNTER(&timer,32767);
  Serial.println(__HAL_TIM_GET_COUNTER(&timer));
  prevTime = micros();

  //Start kapu
  //while(digitalRead(22)!=LOW){delay(1);}
}

void loop() {
  
  //Serial.print("Seb: ");
  //Serial.print(orient);
  //Serial.print("  allapot");
  //Serial.println((int)state);

  //Send interrupt
  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);

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


  //printFrontSensorValue(tresholdedValues);
  // Serial.println("");
  denumFront=0;
  numFront=0;
  
  for(int i=0; i<6; i++){
    for(int j=0; j<8; j++){
      frontSensorPos[i*8+j]=((tresholdedValues[i])<<j);
      frontSensorPos[i*8+j]=frontSensorPos[i*8+j]>>7;
      frontSensorPos[i*8+7]=frontSensorPos[i*8+6];
      if(frontSensorPos[i*8+j]==0){
        forDifference[denumFront]=i*8+j;
        denumFront++;
        numFront=numFront+i*8+j;
      }
    }
  }

  vonalSzam=1;
  for(int i=0; i<denumFront-1; i++){
    if((forDifference[i+1]-forDifference[i])>1){
      vonalSzam++;
    }
  }

  denumBack=0;
  numBack=0;
  
  uint8_t backSensorPos[32]={0};
  for(int i=6; i<10; i++){
    for(int j=0; j<8; j++){
      backSensorPos[(i-6)*8+j]=((tresholdedValues[i])<<j);
      backSensorPos[(i-6)*8+j]=backSensorPos[(i-6)*8+j]>>7;
      backSensorPos[(i-6)*8+7]=backSensorPos[(i-6)*8+6];
      if(backSensorPos[(i-6)*8+j]==0){
        denumBack++;
        numBack=numBack+(i-6)*8+j;
      }
    }
  }

  linePosFront=(((float)numFront/denumFront)-23.5)*0.006;
  linePosBack=(((float)numBack/denumBack)-15.5)*0.006;
  orient=atan((linePosFront-linePosBack)/Lsensor);

  if(isnan(linePosFront)){
    linePosFront=linePosFrontOld;
  }
  
  if(isnan(orient)){
    orient=lastOrient;
  }

 if(isnan(uszog)){
    uszog=uszogOld;
  }
  
  //Serial.println(orient*180/3.14);
  //Serial.println(linePosFront);
  

  /*============================================
  =          Velocity measurement            =
  ============================================*/
  enc_cnt=__HAL_TIM_GET_COUNTER(&timer);
  __HAL_TIM_SET_COUNTER(&timer,32767);
  enc_cnt=enc_cnt-32767;
  tav=enc_cnt*szorzo;
  tav2=tav2+tav;
  int currTime = micros();
  int diffTime = currTime - prevTime;
  prevTime = currTime;
  velo=tav/diffTime*1000000;
  //Serial.println(tav2);
  //Serial.println(velo);

  if(velo==0){
    velo=0.0000000001;
  
  }

  switch (state){
    case 0:
    safetyCar();
    break;
    case 1:
    gyors();
    break;
    case 2:
    lassu();
    break;
    case 3:
    fekez();
    break;
  }

    
   //
  //Serial.println(vonalSzam);
  //Serial.println(timeElapsed);
  lastVonalSzam=vonalSzam;
  lastOrient=orient;
  delay(13);
  //prevVonalSzam=vonalSzam;
  linePosFrontOld=linePosFront;
  uszogOld=uszog;
}


