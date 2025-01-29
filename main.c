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
#include "stm8s.h"
#include "rfm69.h"
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



   uint8_t re_msb =  0;
uint8_t re_lsb =  0;

void main(void)
{
  /* Infinite loop */
  
  
  CLK_HSICmd(ENABLE);//??????RC
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);//HSI?????
  //CLK_SYSCLKDivConfig(CLK_SYSCLKDIV_4);//??????
  CLK_PeripheralClockConfig (CLK_PERIPHERAL_SPI,ENABLE);//??SPI????
  // ----------
  // SPI CONF 
   GPIO_ExternalPullUpConfig(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ENABLE);
   //SPI_IniT(SPI,SPI_FirstBit_MSB,SPI_BaudRatePrescaler_2,SPI_Mode_Master,SPI_CPOL_Low,SPI_CPHA_1Edge,SPI_Direction_2Lines_FullDuplex,SPI_NSS_Soft,0x07);
   SPI_Init( SPI_FIRSTBIT_MSB,
			SPI_BAUDRATEPRESCALER_2,
			SPI_MODE_MASTER,
			SPI_CLOCKPOLARITY_LOW,
			SPI_CLOCKPHASE_1EDGE,
			SPI_DATADIRECTION_2LINES_FULLDUPLEX,
			SPI_NSS_SOFT, 0x07);
   SPI_Cmd(ENABLE);
   
   GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_SLOW); // NSS
    GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_SLOW);
   // ---------------------------------------------
   // GPIO 
    GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
   
   
     
     SPIWrite(0x03,0X22);
   SPIWrite(0x04,0X33);
  
   re_lsb =  SPIRead(0x04);
   re_msb =  SPIRead(0x03);
 //char RxBuffer[150] = {0};
      if (rfm69_init(FREQUENCY, HUB_ID, NETWORKID)) {
    //              GPIO_WriteHigh(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_6); // ININT OK 
      } else {
      //    GPIO_WriteLow(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_6); // FAIL INIT
	}
      
  char data_to_transmit[] = { 0x01, 0x02, 0x03 ,0x04, 0x05, 0x06 ,0x07, 0x08, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x01, 0x02, 0x03 ,0x19,0x20};
      
    // receiveBegin()
  while (1)
  {
    send(DEVICE_ID, data_to_transmit, sizeof(data_to_transmit),FALSE,TRUE);
    //GPIO_ToggleBits(GPIOC, (GPIO_Pin_TypeDef)GPIO_Pin_6);
    //waitForResponce();
    GPIO_WriteReverse(GPIOB,GPIO_PIN_5);
    Delay(0xFFFF);
      Delay(0xFFFF);
       Delay(0xFFFF);
      Delay(0xFFFF);
       Delay(0xFFFF);
      Delay(0xFFFF);
       Delay(0xFFFF);
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
