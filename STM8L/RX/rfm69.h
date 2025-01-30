#ifndef __RFM_H
#define __RFM_H

#include "rfm69reg.h"
#include "stm8l15x.h"

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


#define RFM69_XO               32000000    // /< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 // /< Step width of synthesizer [Hz]
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
// Common ----------------------------------------
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE
// Common ----------------------------------------
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value

#define RF69_TX_LIMIT_MS   1000
// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_BROADCAST_ADDR 255

#define RFM69_RESET_PIN      GPIO_PIN_4
#define RFM69_RESET_PORT     GPIOB

#define SPI_CS_PORT             GPIOB
#define SPI_CS_PIN              GPIO_Pin_3

#define SPI_CS_Low              GPIO_ResetBits(SPI_CS_PORT , SPI_CS_PIN)

#define SPI_CS_High             GPIO_SetBits(SPI_CS_PORT, SPI_CS_PIN)

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


uint8_t waitForResponce(char *data);
uint8_t readData(char *data);

void Delay(uint16_t nCount);


void Delay(uint16_t nCount);

#endif /* __RFM_H */