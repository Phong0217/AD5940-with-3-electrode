#include <Arduino.h>
#include "ad5940.h"
#include <SPI.h>

#define VSPI_MISO_AD5940_Pin   19    //HSPI as 12
#define VSPI_MOSI_AD5940_Pin   23   //HSPI as 13
#define VSPI_SCLK_AD5940_Pin   18   //HSPI as 14
#define VSPI_CS_AD5940_Pin     5   //HSPI as 15
#define AD5940_Reset_Pin       16
#define AD5940_Int_Pin         17

#define LED_PIN                  2 


volatile static uint32_t ucInterrupted = 0;

SPIClass *vspi = NULL;

void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length)
{
  vspi->beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));

  for (int i = 0; i < length; i++)
  {
    *pRecvBuff++ = vspi->transfer(*pSendBuffer++);
  }

  vspi->endTransaction();
}

void AD5940_CsClr(void)
{
    digitalWrite(VSPI_CS_AD5940_Pin, LOW);
}

void AD5940_CsSet(void)
{
    digitalWrite(VSPI_CS_AD5940_Pin, HIGH);
}

void AD5940_RstSet(void)
{
  digitalWrite(AD5940_Reset_Pin, HIGH);
}

void AD5940_RstClr(void)
{
  digitalWrite(AD5940_Reset_Pin, LOW);
}

void AD5940_Delay10us(uint32_t time)
{
  if(time == 0) return;
  delayMicroseconds(10 * time);
}

uint32_t AD5940_GetMCUIntFlag(void)
{
  return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
  ucInterrupted = 0;
  return 1;
}

void Ext_Int0_Handler()
{
  ucInterrupted = 1;
}

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
  /* Step1, initialize SPI peripheral and its GPIOs for CS/RST */
  //start the SPI library (setup SCK, MOSI, and MISO pins)
  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_SCLK_AD5940_Pin, VSPI_MISO_AD5940_Pin, VSPI_MOSI_AD5940_Pin, VSPI_CS_AD5940_Pin);
  //initalize SPI chip select pin
  pinMode(VSPI_CS_AD5940_Pin, OUTPUT);
  //initalize Reset pin
  pinMode(AD5940_Reset_Pin, OUTPUT);

  /* Step2: initialize GPIO interrupt that connects to AD5940's interrupt output */
  //init AD5940 interrupt pin
  pinMode(AD5940_Int_Pin, INPUT_PULLUP);
  //attach ISR for falling edge
  attachInterrupt(digitalPinToInterrupt(AD5940_Int_Pin), Ext_Int0_Handler, FALLING);

  //chip select high to de-select AD5940 initially
  AD5940_CsSet();
  AD5940_RstSet();
  return 0;
}



