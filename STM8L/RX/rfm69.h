/*
 * rfm69.h
 *
 *  Created on: Aug 2, 2021
 *      Author: Vitech-UA
 */

#ifndef INC_RFM69_H_
#define INC_RFM69_H_

#include "stm8l15x.h"
#include "rfm69reg.h"

#define RFM69_SELECT_GPIO RFM_NSEL_GPIO_Port
#define RFM69_SELECT_PIN RFM_NSEL_Pin

#define RFM69_RESET_GPIO RFM_RESET_GPIO_Port
#define RFM69_RESET_PIN RFM_RESET_Pin

#define RFM69_CS_PIN         GPIO_Pin_3
#define RFM69_CS_PORT        GPIOB

#define RFM69_RESET_PIN      GPIO_Pin_4
#define RFM69_RESET_PORT     GPIOB

#define NSS_PIN_RESET GPIO_ResetBits(RFM69_CS_PORT, RFM69_CS_PIN)//GPIOB->ODR &= (uint8_t)(~GPIO_Pin_3)//; 
#define NSS_PIN_SET   GPIO_SetBits(RFM69_CS_PORT, RFM69_CS_PIN)//GPIOB->ODR |= GPIO_Pin_3;

//#define rfm_spi hspi1
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


// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define RF69_MAX_DATA_LEN       61


unsigned char SPICmd8bit(unsigned char WrPara);
unsigned char SPIRead(unsigned char adr);
void SPIWrite(unsigned char adr, unsigned char WrPara); 



bool rfm69_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);
void setMode(uint8_t newMode, bool waitForReady);
void setAddress(uint8_t addr);


void setHighPowerRegs(bool onOff);
void setPowerLevel(uint8_t powerLevel);
int16_t readRSSI(bool forceTrigger);
bool read_data(char *data);
bool waitForResponce(char *data );
void receiveBegin();
uint32_t send(uint8_t toAddress, uint8_t *buffer, uint16_t bufferSize,bool requestACK, bool sendACK);

void Delay(uint16_t nCount);


#endif /* INC_RFM69_H_ */