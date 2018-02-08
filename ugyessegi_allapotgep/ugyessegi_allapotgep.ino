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

int vegTime=0;
int kezdTime=0;
int delaymillis=0;
int statusUart=0;
uint8_t inBuf[2]={0};
double ratioP=0;
double ratioD=0;
double linePosFrontOld=0;
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

double alap=1;
const double constVelo=1;////////////////////////////////////////////Use this to set velocity in m/s
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

float tav_dron_pre=0;
float tav_dron=0;
int i_dron=0;

int baseservo=1500;
int upper=1610;

//Line meas
byte tresholdedValues[10]={0};
float linePosFront=0;
float linePosBack=0;
float orient=0;
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
//0: akadályok között, keresés;
//1: Drón
//2: Sarok
//3: Konvoj
//4: Vasút
//5: Hordó
//6: Körforgalom
//7: Cél
int dronWas=0;
int counterDron=0;
int infraDron=0;
int diffInfraDron=0;


//State-space controller
double kszi=0.85;//sqrt(2)/2;
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

//UART Nucleo es arduino kozott
float ypr0=0;
float ypr1=0;
float infr=0;
float korf=0;

int in_fra_flag=0;
int int_korf=0;
int parkingCntr =0;
int parkingState = 0;
int parkingDir = 0;
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
    /*
    if(velo==0){
      d5=0.5*0.00000001+0.5;
      if(velo<0.6){
        d5=2;
      }
      t5=d5/0.00000001;
      Ttime=(t5*kszi)/3;
      s1s2=(1/(Ttime*Ttime));
      s1pluss2=-2*kszi*(1/Ttime);
      kp=-(Lvesszo/(0.00000001*0.00000001))*s1s2;
      kd=(Lvesszo/0.00000001)*((s1pluss2)-0.00000001*kp);
      uszog=-(-kd*orient-kp*linePosFront)/PI*180;
    }
    else{
      d5=0.5*velo+0.5;
      if(velo<0.6){
        d5=2;
      }
      t5=d5/velo;
      Ttime=(t5*kszi)/3;
      s1s2=(1/(Ttime*Ttime));
      s1pluss2=-2*kszi*(1/Ttime);
      kp=-(Lvesszo/(velo*velo))*s1s2;
      kd=(Lvesszo/velo)*((s1pluss2)-velo*kp);
      uszog=-(-kd*orient-kp*linePosFront)/PI*180;
    }
    */
    /*
    Serial.print("Bemeno szög: ");
    Serial.println(uszog);
    */
    ratioP=250;  ///1000-nél állandó lengés szögben, 800-nél leng, 150-nél még leng egy kicsit
    //ratioD=
    ratioD=1/3;  //Tu=~2  //1/4 volt az eredeti, egyenesre majdnem jó
    //uszog=-(ratioP*linePosFront)/PI*180;
    uszog=(-(ratioP*linePosFront+ratioD*(linePosFront-linePosFrontOld)/0.04));
    
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

void handleFinish(){
  /*enc_cnt=__HAL_TIM_GET_COUNTER(&timer);
  __HAL_TIM_SET_COUNTER(&timer,32767);
  enc_cnt=enc_cnt-32767;
  tavFin=enc_cnt*szorzo;
  tavFin2=tavFin2+tavFin;*/
    servoMotor.writeMicroseconds(basemotor);
    alap=-5;
    
}

void handleParking(){

  /*Állapotaink:
   * 0: Két fal jött, és van vonal
   * 1: megszűnt a vonal
   * 2: Kanyarodunk
   * 3: megyünk előre
   */

  switch (parkingState){
      case 0:
      lineControll();
      alap = 0.6;
      //várunk az első fal végére
      if(analogRead(47)<250 && analogRead(49)<250)
        parkingCntr++;
      if(parkingCntr==10){
        parkingState = 1;
        parkingCntr=0;
      }
      break;
      
      case 1:
      servoMotor.writeMicroseconds(basemotor);
      //várunk a második fal elejére
      alap = 0.6;
      if(analogRead(47)>250 || analogRead(49)>250)
        parkingCntr++;
      if(parkingCntr==10){
        parkingCntr=0;
        parkingState = 2;
        if(analogRead(47)>analogRead(49)){
          parkingDir = 47;
        }else{
          parkingDir = 49;  
        }
      }
      break;
      
      case 2:
      servoMotor.writeMicroseconds(basemotor);
      //várunk a második fal végére
      alap = 0.6;
      if(analogRead(parkingDir)<250)
        parkingCntr++;
      if(parkingCntr==10){
        parkingCntr=0;
        parkingState=3;
      }
      break;
      
      case 3:
        alap = -5;
      break;
      case 4:
      break;
    }
//Inerc szenzor arduino UART
      Serial.print("Inerc:  ");
      HAL_UART_Receive(&uart3struct,(byte*) &ypr0,4,1);
      digitalWrite(38, LOW);
      digitalWrite(15, HIGH);
      delayMicroseconds(200);
      HAL_UART_Receive(&uart3struct,(byte*) &ypr0,4,10);
      HAL_UART_Receive(&uart3struct,(byte*) &ypr1,4,10);
      digitalWrite(38, HIGH);
   
      //Serial.println(ypr0,HEX);
      Serial.print(ypr0*180/3.14,7);
      Serial.print("    ");
      Serial.print(ypr1*180/3.14,7);
      Serial.print("    ");
      Serial.println(timeElapsed);
 /*  
      //Serial.println(ido-timeElapsed);
      ido=timeElapsed;
  
  alap=0.5;
 

  if(vonalSzam!=0){

    if((vonalSzam-lastVonalSzam)<0){
      motorM.writeMicroseconds(1300);
      alap=0;
    }*/

}

void handleDron(){
  dronWas=1;
  Serial.println("beleptem dron");
  if(velo>0){
    alap=-5;
    /*============================================
    =      State-space line following          =
    ============================================*/
    lineControll();
  }

  if(velo==0){
    /*
    Serial.print("Analog");
    Serial.println(analogRead(46));
    */
    
    /*if(counterDron==0){
      infraDron=analogRead(46);

      counterDron=1;
    } 
    else{
        diffInfraDron=infraDron-analogRead(46);
      if((diffInfraDron)>100){

        delay(2100);
        state=0;
      }
    }*/
    if(analogRead(46)<90){
      alap=0;
      delay(2100);
      state = 0;
      u2past=0;
      upast=0;
    }
  }
}

void lineFollowing(){

  if(vonalSzam!=3 && dronWas==0){
    tav_dron=tav_dron-tav; 
    if(tav_dron<0)tav_dron=0;
    Serial.print("Dron tav: " );
    Serial.println(tav_dron, 6);
  }
  
  //Dron -->1
  if(vonalSzam==3 && dronWas==0)
  { 
    tav_dron=tav+tav_dron;
    Serial.print("Dron tav: " );
    Serial.println(tav_dron, 6);
    if(tav_dron>=0.1)state=1;
  }
  
  //Cel -->7
  if (denumFront>15){
    //Cél jön
    state=7;
  }

  if(cycleCount>225){
    if(analogRead(49)>150 && analogRead(47)>150){
      state=2;
      while(statusUart!=HAL_TIMEOUT){
        statusUart=HAL_UART_Receive(&uart3struct,inBuf,1,5);
      }
    }
  }

/*
  Serial.println(linePosFront, 8);
  Serial.println(linePosBack, 8);
  Serial.print("Orient: ");
  Serial.println(orient*180/PI);
*/
  alap=constVelo; 

  lineControll();
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

void InitUart()
{
  __GPIOC_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  
  GPIO_InitTypeDef gpiocstruct;
  gpiocstruct.Alternate = GPIO_AF7_USART3;
  gpiocstruct.Mode=GPIO_MODE_AF_PP;
  gpiocstruct.Pin=pin_rx;
  gpiocstruct.Pull=GPIO_PULLUP;
  gpiocstruct.Speed=GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC,&gpiocstruct);

  GPIO_InitTypeDef gpiocstruct2;
  gpiocstruct2.Alternate = GPIO_AF7_USART3;
  gpiocstruct2.Mode=GPIO_MODE_AF_PP;
  gpiocstruct2.Pin=pin_tx;
  gpiocstruct2.Pull=GPIO_PULLUP;
  gpiocstruct2.Speed=GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB,&gpiocstruct2);
  
  uart3struct.Instance=USART3;
  uart3struct.Init.BaudRate= 38400;
  uart3struct.Init.WordLength=UART_WORDLENGTH_8B;
  uart3struct.Init.StopBits=UART_STOPBITS_1;
  uart3struct.Init.Parity=UART_PARITY_NONE;
  uart3struct.Init.Mode=UART_MODE_TX_RX;
  uart3struct.Init.HwFlowCtl=UART_HWCONTROL_NONE;

  HAL_UART_MspInit (&uart3struct);
  __USART3_CLK_ENABLE();

  HAL_UART_Init(&uart3struct);
}

void setup() {
  Serial.begin(115200);
  Serial.print("started\n");
  Serial.println(kszi);
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
  InitUart();
  
  while(statusUart!=HAL_TIMEOUT){
    statusUart=HAL_UART_Receive(&uart3struct,inBuf,1,5);
  }
  
  delay(1000); ///////////////////////////////////////////////////////////////////////Ez nemtom kell e ide///////////////////
  ido=timeElapsed;

  

  servoMotor.attach(_servopin);
  servoMotor.writeMicroseconds(baseservo);
  motorM.attach(_motorpin);
  motorM.writeMicroseconds(basemotor);

  // give the motor time to set up:
  delay(2000);
  Serial.println("Loop");
  szorzo=(100*PI/(((48*38)/(13*13))*256))/1000;
  __HAL_TIM_SET_COUNTER(&timer,32767);
  Serial.println(__HAL_TIM_GET_COUNTER(&timer));
  

  //Start kapu
  while(digitalRead(22)!=LOW){delay(1);}
  prevTime = micros();
}

void loop() {
  kezdTime=timeElapsed;
  
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
  if(denumFront==0){
    vonalSzam=0;
  }
  denumBack=0;
  numBack=0;
  
  uint8_t backSensorPos[32]={0};
  for(int i=6; i<10; i++){
    for(int j=0; j<7; j++){
      backSensorPos[(i-6)*8+j]=((tresholdedValues[i])<<j);
      backSensorPos[(i-6)*8+j]=backSensorPos[(i-6)*8+j]>>7;
      if(backSensorPos[(i-6)*8+j]==0){
        denumBack++;
        numBack=numBack+(i-6)*8+j;
      }
    }
  }

  linePosFront=(((float)numFront/denumFront)-23.5)*0.006;
  linePosBack=(((float)numBack/denumBack)-15.5)*0.006;
  orient=atan((linePosFront-linePosBack)/Lsensor);

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


  /*
  Serial.print("State");
  Serial.println((int)state);
  Serial.print("Denumfront");
  Serial.println(denumFront);
  */
  //Serial.println("Vonalszam");
  //Serial.println(vonalSzam);
  if(cycleCount<250){
    cycleCount++;
  }
  
  switch (state){
    case 0:
    lineFollowing();
    break;

    case 1:
    handleDron();
    break;
    
    case 2:
    handleParking();
    break;
    
    case 3:
    break;

    case 4:
    break;

    case 5:
    break;

    case 6:
    //handleKorforg();
    break;

    case 7:
    handleFinish();
    break;
  }

    piControll();
//Vonalszam vizsgalat
/*
  if((tav2-temp)>tulHosszu){
    counterTav=0;
  }

  if((vonalSzam-prevVonalSzam)!=0 && vonalSzam != 3){
    if(counterTav==0){
      temp=tav2;
    }
    if(counterTav!=0){
      tavok[prevVonalSzam]=tav2-temp;
      temp=tav2;
    }
    counterTav++;
  }
  if(counterTav==3){
    counterTav=0;
    if(tavok[2]<=0.01){
      //Serial.println("Körforgalom");
    }
    if(tavok[0]<=0.01){
      //Serial.println("Hordo");
    }
  }
 */ 
  linePosFrontOld=linePosFront;

  vegTime=timeElapsed;
  if((vegTime-kezdTime)<20){
    delaymillis=20-(vegTime-kezdTime);
    if(delaymillis<0){
      delaymillis=0;
    }
  }
  delay(delaymillis);
  //Serial.println(timeElapsed);
  //prevVonalSzam=vonalSzam;
}
