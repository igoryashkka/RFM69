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
uint8_t txData[] = {0x99,0x99,0x88,0x88,0x99,0x99}; 
//------------ lib depended AREA 
#define HUB_ID   0xAB
#define DEVICE_ID   0xCB
//------------
#define NETWORKID     0x10  /* (range up to 255)*/
#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY    "key"


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
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
//-------------------------------------------------
// SPI initialization
//-------------------------------------------------
//-------------------------------------------------



void SPI_Init_Config(void) {


  
     SPI_Init(SPI1,SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2, SPI_Mode_Master, SPI_CPOL_Low,SPI_CPHA_1Edge, (SPI_Direction_TypeDef)SPI_Direction_Tx, SPI_NSS_Soft,(uint8_t)0x07);

    SPI_Cmd(SPI1, ENABLE);
}

    
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  /* Infinite loop */ 
  // -------------------------------------------------------------------------
  // Init FOR CLK&SPI&GPIO 
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
  // -------------------------------------------------------------------------
  // Init FOR CLK&SPI&GPIO 
  // -------------------------------------------------------------------------
  
   
 

   /* RESET PIN RFM69 */
   GPIOD->ODR |= GPIO_Pin_3;
   for (uint16_t i = 0; i<1600; i++) nop();      //Delay >100us (for 16 MHz)
   GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_In_FL_No_IT);
   /* RFM 69 reset */
   
   
   
    if (rfm69_init(FREQUENCY, HUB_ID, NETWORKID)) {
		GPIO_SetBits(GPIOC, (GPIO_Pin_TypeDef)GPIO_Pin_6); // ININT OK 
	} else {
		GPIO_ResetBits(GPIOC, (GPIO_Pin_TypeDef)GPIO_Pin_6); // INIT FAIL 
	}


    char data_to_transmit[] = { 0x01, 0x02, 0x03 ,0x04, 0x05, 0x06 ,0x07, 0x08, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x19,0x20};
    

  while (1)
  {
    // TODO : Debouncing & interrupt handling
     if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5 ) == RESET) { 
          GPIO_SetBits(GPIOC, (GPIO_Pin_TypeDef)GPIO_Pin_6);
          send(DEVICE_ID, (uint8_t*) &data_to_transmit, sizeof(data_to_transmit),FALSE,TRUE);
        } else {
           GPIO_ResetBits(GPIOC, (GPIO_Pin_TypeDef)GPIO_Pin_6);
        } 
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
