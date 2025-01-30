/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    30-September-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "rfm69reg.h"
#include "rfm69.h"
    
/** @addtogroup STM8L15x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 char data_to_transmit[] = { 0x01, 0x02, 0x03 ,0x04, 0x05, 0x06 ,0x07, 0x08, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x19,0x20};

//------------ lib depended AREA 
#define HUB_ID   0xAB
#define DEVICE_ID   0xCB
//------------
#define NETWORKID     0x10  /* (range up to 255)*/
#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY    "key"


/* Private function prototypes -----------------------------------------------*/
//#define LED_GPIO_PORT  (GPIOC)
//#define LED_GPIO_PIN GPIO_PIN_0
/* Private functions ---------------------------------------------------------*/
// RFM69 Pins and Ports
#define RFM69_CS_PIN         GPIO_Pin_3
#define RFM69_CS_PORT        GPIOB

#define RFM69_RESET_PIN      GPIO_Pin_4
#define RFM69_RESET_PORT     GPIOB

// RFM69 Registers
#define REG_OPMODE           0x01
#define REG_FIFO             0x00
#define REG_PAYLOAD_LENGTH   0x38
#define REG_IRQ_FLAGS1       0x27
#define REG_IRQ_FLAGS2       0x28

// Modes
#define MODE_STDBY           0x04
#define MODE_TX              0x0C
#define MODE_RX              0x10


void SPI_Init_Config(void) {
    SPI_Init(SPI1,SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2, SPI_Mode_Master, SPI_CPOL_Low,SPI_CPHA_1Edge, (SPI_Direction_TypeDef)SPI_Direction_Tx, SPI_NSS_Soft,(uint8_t)0x07);
    // Enable SPI
    SPI_Cmd(SPI1, ENABLE);
}



                
                

void CheckButtons(void) {
    uint8_t button_state = ~GPIO_ReadInputData(GPIOA) & (GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
    uint8_t button_code = 0;
   // if (DebounceButton(GPIO_Pin_2)) button_state = GPIO_Pin_2;
    //else if (DebounceButton(GPIO_Pin_3)) button_state = GPIO_Pin_3;
    //else if (DebounceButton(GPIO_Pin_4)) button_state = GPIO_Pin_4;
    //else if (DebounceButton(GPIO_Pin_5)) button_state = GPIO_Pin_5;

    switch (button_state) {
        case GPIO_Pin_2:
            GPIO_SetBits(GPIOC, GPIO_Pin_2);
            GPIO_ResetBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
                button_code = 0x01;
            break;
        case GPIO_Pin_3:
            GPIO_SetBits(GPIOC, GPIO_Pin_3);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5);
                button_code = 0x02;
            break;
        case GPIO_Pin_4:
            GPIO_SetBits(GPIOC, GPIO_Pin_4);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5);
                button_code = 0x03;
            break;
        case GPIO_Pin_5:
            GPIO_SetBits(GPIOC, GPIO_Pin_5);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
                 button_code = 0x04;
            break;
        default:
            GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
            break;
    }
        

     if (button_code) {
       data_to_transmit[0] = button_code;
       send(DEVICE_ID, (uint8_t*)&data_to_transmit, sizeof(button_code), FALSE, TRUE);
    }
}


int rfm_freq = 0;
uint8_t rxLength = 0;
uint8_t arrareg[10] = {0};
//uint8_t arraFIFO[31] = {0};
   char re_msb = 0;
   char re_lsb = 0;
     int re_msb_f = 0;
   int re_lsb_f = 0;
    

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  /* Infinite loop */ 
  // -------------------------------------------------------------------------
  // Init FOR SPI&GPIO 
  // -------------------------------------------------------------------------
  
   CLK_HSICmd(ENABLE);
   
   
   CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
   CLK_PeripheralClockConfig(CLK_Peripheral_SPI1,ENABLE);
   CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_8);
   
   GPIO_ExternalPullUpConfig(GPIOB, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, ENABLE);
   
   SPI_Init_Config();
   
   GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_Out_PP_Low_Fast);
   GPIO_Init(GPIOC, GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Fast);
   
   GPIO_Init(GPIOB, GPIO_Pin_3, GPIO_Mode_Out_PP_High_Slow); // NSS
   
   
   GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Fast); // RESET PIN RFM69
   GPIO_Init(GPIOC, GPIO_Pin_0,  GPIO_Mode_In_FL_No_IT); // DIO_0 
   
   GPIO_Init(GPIOA, GPIO_Pin_5, GPIO_Mode_In_PU_No_IT); // Button Pin
   
   
    GPIO_Init(GPIOA, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, GPIO_Mode_In_PU_No_IT); // 4 Buttons PA2-PA5
    GPIO_Init(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Fast); // 4 LEDs 


   
   /* RESET PIN RFM69 */
   GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Fast);
   GPIOD->ODR |= GPIO_Pin_3;
   for (uint16_t i = 0; i<1600; i++) nop();      //Delay >100us (for 16 MHz)
   GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_In_FL_No_IT);
   /* RFM 69 reset */
   
   
   
    if (rfm69_init(FREQUENCY, HUB_ID, NETWORKID)) {
		GPIO_SetBits(GPIOC, (GPIO_Pin_TypeDef)GPIO_Pin_6); // ININT OK 
	} else {
		GPIO_ResetBits(GPIOC, (GPIO_Pin_TypeDef)GPIO_Pin_6); // FAIL INIT
	}


  while (1)
  {

    
    CheckButtons();
    
  
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
 
  /* Infinite loop */
  while (1)
  {
    
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
