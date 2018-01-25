//#define magnetic_encoder_cha 33 // PC8
//#define magnetic_encoder_chb 41 // PB1


//Hajtómotor: TIM1_CH1   PA8
//Servo motor: TIM8_CH4  PC9
#include <Servo.h>
#include <elapsedMillis.h>
#define _motorpin  32  // PC9
#define magnetic_encoder_cha (uint16_t)1<<6 // PC6  Barna  CHB
#define magnetic_encoder_chb (uint16_t)1<<5 // PB5  Piros  CHA

#define pin_tx (uint16_t)1<<10  //PB10
#define pin_rx (uint16_t)1<<5   //PC5

TIM_Encoder_InitTypeDef encoder;
TIM_HandleTypeDef timer;

int enc_cnt=0;
int basemotor=1500;
int upperslow=1600;
int upperfast=1630;
Servo motorM;
double tav=0;
double tav2=0;
double velo=0;
double szorzo=0;
int pulse_count=0;
elapsedMillis timeElapsed;  
int prevTime;
double prevTav = 0;

const int bt_msg_len = 20;

char buf[bt_msg_len];
uint8_t send_buf[bt_msg_len];
uint16_t uartsize=bt_msg_len-1;
uint32_t uarttimeout=10000 ;
uint8_t pData=100;

double n=3.1100000000;

UART_HandleTypeDef uart3struct;

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
  uart3struct.Init.BaudRate= 9600;
  uart3struct.Init.WordLength=UART_WORDLENGTH_8B;
  uart3struct.Init.StopBits=UART_STOPBITS_1;
  uart3struct.Init.Parity=UART_PARITY_NONE;
  uart3struct.Init.Mode=UART_MODE_TX_RX;
  uart3struct.Init.HwFlowCtl=UART_HWCONTROL_NONE;

  HAL_UART_MspInit (&uart3struct);
  __USART3_CLK_ENABLE();

  HAL_UART_Init(&uart3struct);
}

void SendOverBluetooth(uint8_t *msg)
{
  HAL_UART_Transmit(&uart3struct,msg,strlen((const char*)msg),uarttimeout);
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
  Serial.println("Started");

  motorM.attach(_motorpin);
  motorM.writeMicroseconds(basemotor);

  encoderSetup();
  InitUart();
  delay(4000);
  Serial.println("Loop");
  szorzo=(100*PI/(((48*38)/(13*13))*256))/1000;
  __HAL_TIM_SET_COUNTER(&timer,32767);
  prevTime = micros();
}

void loop() {
  motorM.writeMicroseconds(upperslow);
  enc_cnt=__HAL_TIM_GET_COUNTER(&timer);
  __HAL_TIM_SET_COUNTER(&timer,32767);
  enc_cnt=enc_cnt-32767;
  tav=enc_cnt*szorzo;
  tav2=tav2+tav;
  int currTime = micros();
  int diffTime = currTime - prevTime;
  prevTime = currTime;
  velo=tav/diffTime*1000000;
  Serial.print("tav: ");
  Serial.println(tav2);
  Serial.println(velo);
  //Serial.println(diffTime);
  //Serial.println(currTime);
  
  /*
  Serial.print("count: ");
  Serial.println(enc_cnt);
  Serial.println(tav);
  */
  
  String str1=String(diffTime);
  //String str2=String(velo,8);
  String str2=String(velo,8);
  String strSend=str1+","+str2+';'+"\n";
  //Serial.println(strSend);
  strSend.toCharArray(buf,bt_msg_len);
  SendOverBluetooth((uint8_t*)buf);
  delayMicroseconds(1148);
}
