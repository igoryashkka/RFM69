
#include "rfm69.h"



volatile uint8_t _mode;        // current transceiver state
volatile bool _inISR;
volatile uint8_t PAYLOADLEN;

volatile uint8_t DATALEN;
volatile uint8_t SENDERID;
volatile uint8_t TARGETID;
volatile uint8_t PAYLOADLEN;
volatile uint8_t ACK_REQUESTED;
volatile uint8_t ACK_RECEIVED;
volatile uint8_t ctlByte;
volatile int16_t RSSI; // most accurate RSSI during reception (closest to the reception)
volatile bool _inISR;
volatile bool _haveData;

uint8_t _powerLevel;
uint8_t _address;
uint8_t _interruptPin;
uint8_t _interruptNum;
int _reg_val_28 = 0;int _reg_val_27 = 0;

bool _isRFM69HW = FALSE;

void Delay(uint16_t nCount)
{
  while (nCount != 0)
  {
    nCount--;
  }
}



unsigned char SPICmd8bit(unsigned char WrPara)
{ 
  unsigned char RdPara;
  while (SPI_GetFlagStatus(SPI1,SPI_FLAG_TXE) == RESET);
  SPI_SendData(SPI1,WrPara);
  while (SPI_GetFlagStatus(SPI1,SPI_FLAG_RXNE) == RESET);
  RdPara=SPI_ReceiveData(SPI1);
  return RdPara;
}
unsigned char SPIRead(unsigned char adr)
{
  unsigned char data;
  SPI_CS_Low;
  data = SPICmd8bit(adr);              //Send address first
  data = SPICmd8bit(0x00);  
  SPI_CS_High;
  return data;
}
void SPIWrite(unsigned char adr, unsigned char WrPara)  
{
  SPI_CS_Low;						
  SPICmd8bit(adr|0x80);		
  SPICmd8bit(WrPara);          
    SPI_CS_High;
}

bool rfm69_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID) {
      const uint8_t CONFIG[][2] =
			{
			/* 0x01 */{ REG_OPMODE, RF_OPMODE_SEQUENCER_ON| RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
			/* 0x02 */{ REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET| RF_DATAMODUL_MODULATIONTYPE_FSK| RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
            /* 0x03 */{ REG_BITRATEMSB, RF_BITRATEMSB_4800 }, // default: 4.8 KBPS
			/* 0x04 */{ REG_BITRATELSB, RF_BITRATEMSB_4800 },
			/* 0x05 */{ REG_FDEVMSB, RF_FDEVMSB_75000 }, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
			/* 0x06 */{ REG_FDEVLSB, RF_FDEVLSB_75000 },

			/* 0x07 */{ REG_FRFMSB, (uint8_t) (freqBand == RF69_315MHZ ?RF_FRFMSB_315 :(freqBand == RF69_433MHZ ?RF_FRFMSB_433 :(freqBand == RF69_868MHZ ?RF_FRFMSB_868 :RF_FRFMSB_915))) },
			/* 0x08 */{ REG_FRFMID, (uint8_t) (freqBand == RF69_315MHZ ?RF_FRFMID_315 :(freqBand == RF69_433MHZ ?RF_FRFMID_433 :(freqBand == RF69_868MHZ ?RF_FRFMID_868 :RF_FRFMID_915))) },
                        /* 0x09 */{ REG_FRFLSB, (uint8_t) (freqBand == RF69_315MHZ ?RF_FRFLSB_315 :(freqBand == RF69_433MHZ ?RF_FRFLSB_433 :(freqBand == RF69_868MHZ ?RF_FRFLSB_868 :RF_FRFLSB_915))) },

					// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
					// +17dBm and +20dBm are possible on RFM69HW
					// +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
					// +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
					// +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
				  { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF| RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111 },
			/* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

					// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
			/* 0x19 */{ REG_RXBW, RF_LNA_GAINSELECT_MAXMINUS12 | RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16| RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
                        
					//for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
					/* 0x25 */{ REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
					/* 0x26 */{ REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
					/* 0x28 */{ REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
					/* 0x29 */{ REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
				        /* 0x2D */ { REG_PREAMBLELSB, 0x80 }, // default 3 preamble bytes 0xAAAAAA
					/* 0x2E */{ REG_SYNCCONFIG, RF_SYNC_ON
							| RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2
							| RF_SYNC_TOL_0 },
					// /* 0x2F */{ REG_SYNCVALUE1, 0x2D }, // attempt to make this compatible with sync1 byte of RFM12B lib
					/* 0x30 */{ REG_SYNCVALUE2, networkID }, // NETWORK ID
					/* 0x37 */{ REG_PACKETCONFIG1,RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF| RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON| RF_PACKET1_ADRSFILTERING_OFF },
					/* 0x38 */{ REG_PAYLOADLENGTH, 60}, // in variable length mode: the max frame size, not used in TX
					// /* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
					/* 0x3C */{ REG_FIFOTHRESH,RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
					/* 0x3D */{ REG_PACKETCONFIG2,RF_PACKET2_RXRESTARTDELAY_2BITS| RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
					//for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
					/* 0x6F */{ REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
					{ 255, 0 } };


     
   do {SPIWrite(REG_SYNCVALUE1,0xAA);} while (SPIRead(REG_SYNCVALUE1) != 0xaa);
   do {SPIWrite(REG_SYNCVALUE1, 0x55);} while (SPIRead(REG_SYNCVALUE1) != 0x55);
                                    
	for (uint8_t i = 0; CONFIG[i][0] != 255; i++) {
		SPIWrite(CONFIG[i][0], CONFIG[i][1]);
	}

    setMode(RF69_MODE_STANDBY, FALSE);

	while (((SPIRead(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)); // wait for ModeReady

    setHighPowerRegs(TRUE);
      setPowerLevel(13);
	setAddress(nodeID);
        
        
	return TRUE;
}



void setHighPowerRegs(bool onOff) {
	SPIWrite(REG_TESTPA1, onOff ? 0x5D : 0x55);
	SPIWrite(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

void setPowerLevel(uint8_t powerLevel) {
	_powerLevel = (powerLevel > 31 ? 31 : _powerLevel);
	if (_isRFM69HW)
		_powerLevel /= 2;
	SPIWrite(REG_PALEVEL, (SPIRead(REG_PALEVEL) & 0xE0) | _powerLevel);
}


void setAddress(uint8_t addr) {
	_address = addr;
	SPIWrite(REG_NODEADRS, _address);
}


void setMode(uint8_t newMode, bool waitForReady) {
	if (newMode == _mode)
		return;

	switch (newMode) {
	case RF69_MODE_TX:
		SPIWrite(REG_OPMODE,(SPIRead(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
		break;
	case RF69_MODE_RX:
		SPIWrite(REG_OPMODE, (SPIRead(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
		break;
	case RF69_MODE_SYNTH:
		SPIWrite(REG_OPMODE,(SPIRead(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
		break;
	case RF69_MODE_STANDBY:
		SPIWrite(REG_OPMODE, (SPIRead(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
		break;
	case RF69_MODE_SLEEP:
		SPIWrite(REG_OPMODE, (SPIRead(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
		break;
	default:
		return;
	}

	if (waitForReady) {
		while ((SPIRead(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
	}

	_mode = newMode;
}




// -------------------- SEND FUNC START  -------------------- // 
uint32_t send(uint8_t toAddress, uint8_t *buffer, uint16_t bufferSize,bool requestACK, bool sendACK) {
	setMode(RF69_MODE_STANDBY, TRUE); // turn off receiver to prevent reception while filling fifo
	SPIWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"

	if (bufferSize > 61) {
		bufferSize = 61;
	}

	
        SPIWrite(REG_FIFO, bufferSize);

        for (uint8_t i = 0; i < bufferSize; i++) {
		SPIWrite(REG_FIFO,buffer[i]);
	}
       
        
	// no need to wait for transmit mode to be ready since its handled by the radio
	setMode(RF69_MODE_TX, TRUE);

     
	//while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == RESET); // wait UP DIO_0 - transiving is over 
      while (!(SPIRead(REG_IRQFLAGS2) & (1 << 3) )) {}; // wait for DIO0 to be high

	setMode(RF69_MODE_STANDBY,TRUE);


	return 1;
}
// -------------------- SEND FUNC END  -------------------- // 



void receiveBegin() {
	if (SPIRead(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
		SPIWrite(REG_PACKETCONFIG2,(SPIRead(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	}
	//SPIWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
	setMode(RF69_MODE_RX, FALSE);
}


uint8_t waitForResponce(char *data){

   while (!(SPIRead(REG_IRQFLAGS2) & (1 << 2))) {}; // // PAYLOADREADY

   return readData(data);
}


   uint8_t arr_test[10] = {0};

uint8_t readData(char *data) {
	//if (_mode == RF69_MODE_RX && (SPIRead(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {

		setMode(RF69_MODE_STANDBY, TRUE);
	
        for (uint8_t i = 0; i < 10; i++) {
		    arr_test[i] = SPIRead(0x00);
	    }

        nop();
		setMode(RF69_MODE_RX, TRUE);


	return arr_test[1];
}
