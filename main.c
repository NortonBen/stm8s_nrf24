/**
  ******************************************************************************
  * @file    main.c 
  * @author  NortonBen
  * @version V1.0.0
  * @date    14-01-2024
  * @brief   Main program body
   ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "nRF24.h"

/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void main(void)
{
  
  Nrf24l_init();
  Nrf24l_setRADDR("node1");
  Nrf24l_setTADDR("node2");
  Nrf24l_config(120, 32);
  
  /* Infinite loop */
  uint8_t i = 0;
  uint8_t t = 0;
  uint8_t data[32];
  while (1)
  {
    
      while(!Nrf24l_dataReady()){
          uint16_t delay = 0xFF;
          while(delay--);
          asm("nop");
      }
      
      Nrf24l_getData(data);
      
     for(t =0 ; t < 20; t++) {
      for(i =0 ; i < 255; i++) {
          uint16_t delay = 0xFF;
          while(delay--) {
             asm("nop");
          }
      }
     }
    
      for(i = 0; i < 32; i ++) {
        data[i] = 0;
      }

      data[0] = 2;
      
      Nrf24l_send(data);
      while(Nrf24l_isSending()){
         asm("nop");
      }
      
      for(i =0 ; i < 255; i++) {
          uint16_t delay = 0xFF;
          while(delay--);
      }

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
