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

//Sensor's memory register addresses:
const int PRESSURE = 0x1F;      //3 most significant bits of pressure
const int PRESSURE_LSB = 0x20;  //16 least significant bits of pressure
const int TEMPERATURE = 0x21;   //16 bit temperature reading
const byte READ = 0b11111100;     // SCP1000's read command
const byte WRITE = 0b00000010;   // SCP1000's write command
const int delay_us = 2000;

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int dataReadyPin = 6;
const int chipSelectPin = 5;

void setup() {
  Serial.begin(9600);
  Serial.print("started\n");

  // start the SPI library:
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));

  // initalize the  data ready and chip select pins:
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);
  pinMode(44, OUTPUT);

  //Configure SCP1000 for low noise configuration:
  //writeRegister(0x02, 0x2D);
  //writeRegister(0x01, 0x03);
  //writeRegister(0x03, 0x02);
  // give the sensor time to set up:
  delay(100);
  Serial.println("Loop");
}

void loop() {
      
  digitalWrite(44,HIGH);
  delay(1);
  digitalWrite(44,LOW);
  delay(5);

  byte pos[8] = {0};
  byte ans[8] = {0};

  for(int i = 0; i < 8; i++)
  {
    digitalWrite(chipSelectPin, LOW);
    delayMicroseconds(delay_us);
  
    pos[i] = SPI.transfer(i);
    delayMicroseconds(delay_us);
    ans[i] = SPI.transfer(0xAA);
    digitalWrite(chipSelectPin, HIGH);
    delayMicroseconds(delay_us);  
  }

  printSensorValue(ans, pos);
}

void printSensorValue(byte value[8],byte pos[8]){
  Serial.print("\n\n");

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



