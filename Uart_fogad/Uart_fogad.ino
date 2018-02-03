#include "String.h"
#include <elapsedMillis.h>

#define pin_tx (uint16_t)1<<10  //PB10
#define pin_rx (uint16_t)1<<5   //PC5

const int bt_msg_len = 20;

elapsedMillis timeElapsed;


char buf[bt_msg_len];
uint8_t send_buf[bt_msg_len];
uint16_t uartsize=bt_msg_len-1;
uint32_t uarttimeout=10000 ;
uint8_t pData=100;

uint8_t inBuf[14]={0};
char zChar[6];
String zStr;
double zAxis=0;
char yChar[6];
String yStr;
double yAxis=0;
double ido=0;

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
  Serial.println("Started");
  pinMode(38 ,OUTPUT);
  InitUart();
  delay(4000);
  ido=timeElapsed;
}

void loop() {
  digitalWrite(38, HIGH);
  delayMicroseconds(200);
  HAL_UART_Receive(&uart3struct,inBuf,14,10);
  digitalWrite(38, LOW);

  for(int i=0;i<6;i++){
    zChar[i]=inBuf[i+1];
    yChar[i]=inBuf[i+8];
  }
  Serial.println(inBuf[0]);
 
  zStr=String(zChar);
  zAxis=zStr.toFloat();
  yStr=String(yChar);
  yAxis=yStr.toFloat();
  Serial.println(zAxis);
  Serial.println(yAxis);
  Serial.println(ido-timeElapsed);
  ido=timeElapsed;
  
  
}

