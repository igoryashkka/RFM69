/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.3.0
  * @date    16-June-2017
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
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define NETWORKID     0x10  /* (range up to 255)*/
#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY    "key"
#define HUB_ID   0xAB
   uint8_t re_msb =  0;
uint8_t re_lsb =  0;
    uint8_t button_state;
    uint8_t button_code;

void main(void)
{
  /* Infinite loop */
  
  
  CLK_HSICmd(ENABLE);//??????RC
CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
   CLK_PeripheralClockConfig(CLK_Peripheral_SPI1,ENABLE);
      CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_8);
  
  // ----------
  // SPI CONF 
 GPIO_ExternalPullUpConfig(GPIOB, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, ENABLE);
  
   //SPI_IniT(SPI,SPI_FirstBit_MSB,SPI_BaudRatePrescaler_2,SPI_Mode_Master,SPI_CPOL_Low,SPI_CPHA_1Edge,SPI_Direction_2Lines_FullDuplex,SPI_NSS_Soft,0x07);
  SPI_Init(   SPI1,
                SPI_FirstBit_MSB,
                SPI_BaudRatePrescaler_2,
                SPI_Mode_Master,
                SPI_CPOL_Low,
                SPI_CPHA_1Edge,
                SPI_Direction_2Lines_FullDuplex,
                SPI_NSS_Soft,
                0x07);
   SPI_Cmd(SPI1,ENABLE);
   
     GPIO_Init(GPIOB, GPIO_Pin_3, GPIO_Mode_Out_PP_High_Slow); // NSS
   
   
   GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Fast);
   // ---------------------------------------------
   // GPIO 
    GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_Out_PP_Low_Fast);
    GPIO_Init(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Fast); // 4 LED
   //  GPIO_Init(GPIOD, GPIO_Pin_4, GPIO_MODE_OUT_PP_LOW_FAST);
   

  char RxBuffer[150] = {0};
  
  
   /* RESET PIN RFM69 */
   GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Fast);
   GPIOD->ODR |= GPIO_Pin_3;
   for (uint16_t i = 0; i<1600; i++) nop();      //Delay >100us (for 16 MHz)
   GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_In_FL_No_IT);
   /* RFM 69 reset */
   
   
  
      if (rfm69_init(FREQUENCY, HUB_ID, NETWORKID)) {
    //              GPIO_WriteHigh(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_6); // ININT OK 
      } else {
      //    GPIO_WriteLow(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_6); // FAIL INIT
	}
      
  //char data_to_transmit[] = { 0x01, 0x02, 0x03 ,0x04, 0x05, 0x06 ,0x07, 0x08, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x19,0x20};
      
    receiveBegin();
  while (1)
  {
  
    button_state = waitForResponce(RxBuffer);
    switch (button_state) {
        case 0x01:
            GPIO_SetBits(GPIOC, GPIO_Pin_2);
            GPIO_ResetBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
                button_code = 0x01;
            break;
        case 0x02:
            GPIO_SetBits(GPIOC, GPIO_Pin_3);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5);
                button_code = 0x02;
            break;
        case 0x03:
            GPIO_SetBits(GPIOC, GPIO_Pin_4);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5);
                button_code = 0x03;
            break;
        case 0x04:
            GPIO_SetBits(GPIOC, GPIO_Pin_5);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
                 button_code = 0x04;
            break;
        default:
            //GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
            break;
    }
    nop();
    Delay(0xFFFF);


  }
  
  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
