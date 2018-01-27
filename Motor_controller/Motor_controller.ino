#include <Servo.h>
#include <elapsedMillis.h>
#define _motorpin  32  // PC9
#define magnetic_encoder_cha (uint16_t)1<<6 // PC6  Barna  CHB
#define magnetic_encoder_chb (uint16_t)1<<5 // PB5  Piros  CHA


TIM_Encoder_InitTypeDef encoder;
TIM_HandleTypeDef timer;

int enc_cnt=0;

int basemotor=1500;
int upperslow=1700; //////////////////////////////////////////////////////////

Servo motorM;
double tav=0;
double tav2=0;
double velo=0;
double szorzo=0;
int pulse_count=0;
elapsedMillis timeElapsed;  
int prevTime;
double prevTav = 0;

int motorFord=1500;
double Kc=1;//0.6158
double zd=0.9678;

double u1=0;
double u2=0;
double u=0;
double u2past=0;
double upast=0;

double alap=5.89;
double beta=65.848;
double alfa=0.0422;

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
  Serial.println("Started");

  motorM.attach(_motorpin);
  motorM.writeMicroseconds(basemotor);

  encoderSetup();
  delay(4000);
  Serial.println("Loop");
  szorzo=(100*PI/(((48*38)/(13*13))*256))/1000;
  __HAL_TIM_SET_COUNTER(&timer,32767);
  prevTime = micros();

}

void loop() {

  enc_cnt=__HAL_TIM_GET_COUNTER(&timer);
  __HAL_TIM_SET_COUNTER(&timer,32767);
  enc_cnt=enc_cnt-32767;
  tav=enc_cnt*szorzo;
  tav2=tav2+tav;
  int currTime = micros();
  int diffTime = currTime - prevTime;
  prevTime = currTime;
  velo=tav/diffTime*1000000;
  //Sebességérzékelése, és kivonás az alapjelből
 u2=zd*u2past+(1-zd)*upast;
 u1=Kc*(alap-velo);
 u=(u1+u2);
 if(u>6)
  u=6;
 if(u<1)
  u=1;
 motorFord=(u+beta)/alfa;
 Serial.println(motorFord);
 Serial.println(velo);
 motorM.writeMicroseconds(motorFord);
 u2past=u2;
 upast=u; 

}
