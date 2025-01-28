#ifndef __RFM_H
#define __RFM_H

#include "rfm69reg.h"
#include "stm8s.h"

#define MODE_STDBY           0x04
#define MODE_TX              0x0C
#define MODE_RX              0x10


#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91
#define HUB_ID   0xAB
#define DEVICE_ID   0xCB
//------------
#define NETWORKID     0x10  
#define FREQUENCY   RF69_868MHZ

/* (range up to 255)*/
//#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY    "key"

#define SPI_CS_PORT             GPIOA
#define SPI_CS_PIN              GPIO_PIN_3

#define RFM69_RESET_PIN      GPIO_PIN_4
#define RFM69_RESET_PORT     GPIOB

/* ??SPI????,??CS???? */
#define SPI_CS_Low              GPIO_WriteLow(SPI_CS_PORT , SPI_CS_PIN)
/* ??SPI????,??CS???? */
#define SPI_CS_High             GPIO_WriteHigh(SPI_CS_PORT, SPI_CS_PIN)

unsigned char SPICmd8bit(unsigned char WrPara);
unsigned char SPIRead(unsigned char adr);
void SPIWrite(unsigned char adr, unsigned char WrPara);
bool rfm69_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);
uint32_t send(uint8_t toAddress, uint8_t *buffer, uint16_t bufferSize,bool requestACK, bool sendACK);


void setHighPowerRegs(bool onOff);
void setPowerLevel(uint8_t powerLevel);
void setAddress(uint8_t addr);
void setMode(uint8_t newMode, bool waitForReady);
void receiveBegin();


void waitForResponce(char *data);
bool readData(char *data);

void Delay(uint16_t nCount);

#endif /* __RFM_H */