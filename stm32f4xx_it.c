/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm_filter.h"
#include "stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t InternalBufferSize = 0;
uint32_t Data_Status = 0;
int x = 0;
int y = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

/*void SysTick_Handler(void)
{
} */


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
void SPI2_IRQHandler(void){
  extern PDMFilter_InitStruct Filter;
  extern uint16_t* AudioRecBuf;
  extern uint16_t* WriteBuf;
  extern uint16_t  PDM_Input_Buffer[];
  extern uint16_t PCM_Output_Buffer[];

  u16 volume;
  u16 app;

  // Check if new data are available in SPI data register
  if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET){
    // Read received data and save it in internal table
    app = SPI_I2S_ReceiveData(SPI2);
   // PDM_Input_Buffer[InternalBufferSize++] = (uint8_t)app;
    PDM_Input_Buffer[InternalBufferSize++] = /*(uint8_t)*/HTONS(app);

    // Check to prevent overflow condition
    if (InternalBufferSize >= PDM_Input_Buffer_SIZE){
      InternalBufferSize = 0;

      volume = 50;

      PDM_Filter_64_LSB((uint8_t *)PDM_Input_Buffer, (uint16_t *)AudioRecBuf/*PCM_Output_Buffer*/, volume, &Filter);

      Data_Status = 1;
    }
  }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
