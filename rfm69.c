
#include "rfm69.h"

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
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}


unsigned char SPICmd8bit(unsigned char WrPara)
{ 
  unsigned char RdPara;
  while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
  SPI_SendData(WrPara);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  RdPara=SPI_ReceiveData();
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
  //unsigned char data;
  SPI_CS_Low;						
  SPICmd8bit(adr|0x80);		//???????
  SPICmd8bit(WrPara);           //????
  SPI_CS_High;
}

bool rfm69_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID) {
  
      //rfm69_up_reset_pin();
      //Delay_LIB(0xffff);
      //rfm69_down_reset_pin();
    // magic = SX1278Read(0x01); //  MAGIC READ
    
	const uint8_t CONFIG[][2] =
			{
			/* 0x01 */{ REG_OPMODE, RF_OPMODE_SEQUENCER_ON| RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
			/* 0x02 */{ REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET| RF_DATAMODUL_MODULATIONTYPE_FSK| RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
                        /* 0x03 */{ REG_BITRATEMSB, RF_BITRATEMSB_4800 }, // default: 4.8 KBPS
			/* 0x04 */{ REG_BITRATELSB, RF_BITRATEMSB_4800 },
			/* 0x05 */{ REG_FDEVMSB, RF_FDEVMSB_300000 }, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
			/* 0x06 */{ REG_FDEVLSB, RF_FDEVLSB_300000 },

			/* 0x07 */{ REG_FRFMSB, (uint8_t) (freqBand == RF69_315MHZ ?RF_FRFMSB_315 :(freqBand == RF69_433MHZ ?RF_FRFMSB_433 :(freqBand == RF69_868MHZ ?RF_FRFMSB_868 :RF_FRFMSB_915))) },
			/* 0x08 */{ REG_FRFMID, (uint8_t) (freqBand == RF69_315MHZ ?RF_FRFMID_315 :(freqBand == RF69_433MHZ ?RF_FRFMID_433 :(freqBand == RF69_868MHZ ?RF_FRFMID_868 :RF_FRFMID_915))) },
                        /* 0x09 */{ REG_FRFLSB, (uint8_t) (freqBand == RF69_315MHZ ?RF_FRFLSB_315 :(freqBand == RF69_433MHZ ?RF_FRFLSB_433 :(freqBand == RF69_868MHZ ?RF_FRFLSB_868 :RF_FRFLSB_915))) },

					// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
					// +17dBm and +20dBm are possible on RFM69HW
					// +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
					// +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
					// +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
				  { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_ON| RF_PALEVEL_PA2_ON | RF_PALEVEL_OUTPUTPOWER_11111 },
			/* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

					// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
			/* 0x19 */{ REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16| RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
                        
					//for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
					/* 0x25 */{ REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
					/* 0x26 */{ REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
					/* 0x28 */{ REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
					/* 0x29 */{ REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
				        /* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE }, // default 3 preamble bytes 0xAAAAAA
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
                                    

        //reg_val_28 = SPIRead(0x28);
        //reg_val_27 = SPIRead(0x27);
        
	for (uint8_t i = 0; CONFIG[i][0] != 255; i++) {
		SPIWrite(CONFIG[i][0], CONFIG[i][1]);
	}
        
        //        reg_val_MSB = SPIRead(0x03);
        //reg_val_LSB = SPIRead(0x04);
       // reg_val_28 = SPIRead(0x28);
       // reg_val_27 = SPIRead(0x27);

    setMode(RF69_MODE_STANDBY, FALSE);
        
     //      reg_val_28 = SPIRead(0x28);
      //  reg_val_27 = SPIRead(0x27);

	while (((SPIRead(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)); // wait for ModeReady

    setHighPowerRegs(TRUE);
    setPowerLevel(25);
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
		//if (_isRFM69HW)       //  can be unused for RFM69CW
		//setHighPowerRegs(true); //  can be unused for RFM69CW
		break;
	case RF69_MODE_RX:
		SPIWrite(REG_OPMODE, (SPIRead(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
		//if (_isRFM69HW)            //  can be unused for RFM69CW
		//	setHighPowerRegs(FALSE); //  can be unused for RFM69CW
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

        _reg_val_28 = SPIRead(0x28);
        _reg_val_27 = SPIRead(0x27);
        
        
	if (waitForReady) {
		while ((SPIRead(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
	}

	_mode = newMode;
}

uint8_t buff_test[10] = {0};



// -------------------- SEND FUNC START  -------------------- // 
uint32_t send(uint8_t toAddress, uint8_t *buffer, uint16_t bufferSize,bool requestACK, bool sendACK) {
	setMode(RF69_MODE_STANDBY, TRUE); // turn off receiver to prevent reception while filling fifo
	SPIWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"

	// control byte
	if (bufferSize > 61) {
		bufferSize = 61;
	}


	
        SPIWrite(REG_FIFO, bufferSize);
	// write to FIFO

        for (uint8_t i = 0; i < bufferSize; i++) {
		SPIWrite(REG_FIFO,buffer[i]);
	}

        _reg_val_28 = SPIRead(0x28);
        //for (uint8_t i = 0; i < bufferSize; i++) {
	//	buff_test[i] = SPIRead(REG_FIFO);
	//}
       
        
	// no need to wait for transmit mode to be ready since its handled by the radio
	setMode(RF69_MODE_TX, TRUE);

       _reg_val_28 = SPIRead(0x28);
             _reg_val_27 = SPIRead(0x27);
	 _reg_val_27 = SPIRead(0x1);
	//while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == RESET); // wait UP DIO_0 - transiving is over  PacketSent
      while (!(SPIRead(REG_IRQFLAGS2) & (1 << 3) )) {}; // wait for DIO0 to be high // PacketSent
        //clear_fifo();
        _reg_val_28 = SPIRead(0x28);
         _reg_val_27 = SPIRead(0x27);
	 _reg_val_27 = SPIRead(0x1);
        _reg_val_27 = SPIRead(0x1);
	setMode(RF69_MODE_STANDBY,TRUE);
	//receiveBegin();

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


void waitForResponce(char *data){

    while (!(SPIRead(REG_IRQFLAGS2) & (1 << 2))) {}; // // PAYLOADREADY

    readData(data);
}




bool readData(char *data) {
	if (_mode == RF69_MODE_RX && (SPIRead(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {

		//data->targetId = data->senderId = data->ctlByte = 0xFF;

		//data->signalStrength = readRSSI(FALSE);
		//memset(data->data, 0, RF69_MAX_DATA_LEN);

		// Читаю кадр
		setMode(RF69_MODE_STANDBY, /*waitForReady=*/TRUE);
		//uint8_t zero_byte = 0;
		//uint8_t read_data = REG_FIFO & 0x7F;

		//rfm69_select();
		//HAL_SPI_Transmit(&rfm_spi, &read_data, 1, 100);
		//HAL_SPI_TransmitReceive(&rfm_spi, (uint8_t*) &zero_byte,
		//		(uint8_t*) &data->size, 1, 100);

		//HAL_StatusTypeDef errorCode = HAL_SPI_Receive(&RFM69_SPI_PORT, *&frame,
		//		data->size, HAL_MAX_DELAY);

        for (uint8_t i = 0; i < 10; i++) {
		    *data = SPIRead(0x00);
	    }


		//rfm69_release();
		setMode(RF69_MODE_RX, TRUE);

		//Парсим кадр
            /*
		if (errorCode == HAL_OK) {
			data->targetId = frame[0];
			data->senderId = frame[1];
			data->ctlByte = frame[2];
			for (int8_t i = 3; i < data->size; i++) {
				data->data[i - 3] = frame[i];
			}
			writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
			setMode(RF69_MODE_RX, false);
			return true;
		}*/
	}
	return FALSE;
}