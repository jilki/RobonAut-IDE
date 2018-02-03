#define pin_tx (uint16_t)1<<1 //PA2
#define pin_rx (uint16_t)1<<0   //PA3


uint8_t inBuf[2]={9};

UART_HandleTypeDef uart2struct;

void InitUart()
{
  __GPIOA_CLK_ENABLE();
  
  GPIO_InitTypeDef gpiocstruct;
  gpiocstruct.Alternate = GPIO_AF7_USART2;
  gpiocstruct.Mode=GPIO_MODE_AF_PP;
  gpiocstruct.Pin=pin_rx;
  gpiocstruct.Pull=GPIO_PULLUP;
  gpiocstruct.Speed=GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA,&gpiocstruct);

  GPIO_InitTypeDef gpiocstruct2;
  gpiocstruct2.Alternate = GPIO_AF7_USART2;
  gpiocstruct2.Mode=GPIO_MODE_AF_PP;
  gpiocstruct2.Pin=pin_tx;
  gpiocstruct2.Pull=GPIO_PULLUP;
  gpiocstruct2.Speed=GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA,&gpiocstruct2);
  
  uart2struct.Instance=USART2;
  uart2struct.Init.BaudRate= 115200;
  uart2struct.Init.WordLength=UART_WORDLENGTH_8B;
  uart2struct.Init.StopBits=UART_STOPBITS_1;
  uart2struct.Init.Parity=UART_PARITY_NONE;
  uart2struct.Init.Mode=UART_MODE_RX; //UART_MODE_TX_RX
  uart2struct.Init.HwFlowCtl=UART_HWCONTROL_NONE;

  HAL_UART_MspInit (&uart2struct);
  __USART2_CLK_ENABLE();

  HAL_UART_Init(&uart2struct);
}


void setup() {
  //Serial.begin(115200);
//  Serial.println("None");
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  InitUart();
  delay(1000);
  //Serial.println("Started");
}

void loop() {
  delayMicroseconds(200);
  HAL_UART_Receive(&uart2struct,inBuf,1,10);
  if(inBuf[0]!=0){
    //Serial.println(inBuf[0]);
  }
  else{
    //Serial.println("START!!!");
    digitalWrite(7, HIGH);
  }
}
