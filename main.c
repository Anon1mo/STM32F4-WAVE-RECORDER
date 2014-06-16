/*
 * ------ STM32F4 Discovery WAVE(.wav) recorder ------
 * 		by Piotr Jakubowski and Filip Skurniak
 *
 *  We used ST example: http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00040802.pdf
 *
 *  You can record and save files on your computer using STM32F4 Discovery.
 *
 *  What you need:
 *  CooCox CoIDE
 *  The STM32F4xx Discovery board has a built in [ST-LINK/V2 in-circuit debugger/programmer](http://www.st.com/web/catalog/tools/FM146/CL1984/SC724/SS1677/PF251168).
 *  [Virtual Com Port driver](http://www.st.com/web/en/catalog/tools/PF257938) so you can talk to your STM32F4 once you load this project.
 *  The C# program, which you can download from the site of our project in GitHub
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include "arm_math.h"               // Required to use float32_t type, provides FFT functions
#include "main.h"

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"

#include "pdm_filter.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lcd.h"
#include "stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t  PDM_Input_Buffer[PDM_Input_Buffer_SIZE]; // buffer for PDM samples
uint16_t PCM_Output_Buffer[PCM_Output_Buffer_SIZE]; // 1st buffer for PCM samples
uint16_t PCM_Output_Buffer1[PCM_Output_Buffer_SIZE]; // 2nd buffer for PCM samples(used when data from first is being saved)
uint16_t buffer_input[Buffer_Input_SIZE]; // 1st buffer that aggregates PCM samples
uint16_t buffer_input1[Buffer_Input_SIZE]; // 2nd buffer that aggregates PCM samples(used when the first is busy)

uint16_t buf_idx = 0, buf_idx1 =0;

uint16_t* AudioRecBuf; // pointer to Audio Recording Buffer, it's set to PCM_Output_Buffer or PCM_Output_Buffer2
uint16_t* WriteBuf; // pointer to buffer, from which bytes are being sent

float32_t maxvalue; // in case you use FFT
uint32_t  maxvalueindex; // in case you use FFT
uint8_t   mode;
float32_t f;
float32_t f2;
int sw = 0; // var to switch buffers

char      text[100];

arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;
PDMFilter_InitStruct Filter;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configure(void);
static void I2S_Configure(void);
static void NVIC_Configure(void);
static void RCC_Configure(void);
void SysTick_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void OTG_FS_IRQHandler(void);
void OTG_FS_WKUP_IRQHandler(void);
/*USB------------------------------------------------------------------------*/
volatile uint32_t ticker, downTicker;

/*
 * The USB data must be 4 byte aligned if DMA is enabled. This macro handles
 * the alignment, if necessary (it's actually magic, but don't tell anyone).
 */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/* Functions -----------------------------------------------------------------*/
int main(void)
{
	extern uint32_t Data_Status; // Data_Status indicates whether data is ready to be sent



	unsigned int freq, i, j;// z;
	long z;
	float dt;

	RCC_Configure();
	NVIC_Configure();
	GPIO_Configure();
	I2S_Configure();


	  // Initialize PDM filter
	  Filter.Fs = 16000;
	  Filter.HP_HZ = 10;
	  Filter.LP_HZ = 8000;
	  Filter.In_MicChannels = 1;
	  Filter.Out_MicChannels = 1;
	  PDM_Filter_Init(&Filter);

	 // Initialize FFT for 2048 samples
	arm_rfft_init_f32(&S, &S_CFFT, 2048, 0, 1);

	// Initialize User Button
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);

	// If everything is initialized then the two diodes are on
	GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
	while(STM_EVAL_PBGetState(BUTTON_USER) == RESET);
	while(STM_EVAL_PBGetState(BUTTON_USER) == SET);

	// Press the USER BUTTON and everything is ready from now on
	GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);

	while (1)
	{
	    dt = 0.001;
	    freq = 0;
	    mode = 1;


	    z = 0;

		/* Blink the orange LED at 1Hz */
		if (500 == ticker)
		{
				GPIOD->BSRRH = GPIO_Pin_13;
		}
		else if (1000 == ticker)
		{
			ticker = 0;
			GPIOD->BSRRL = GPIO_Pin_13;
		}


		/* If there's data on the virtual serial port:
		 *  - Echo it back
		 *  - Turn the green LED on for 10ms
		 */
		uint16_t theByte;


		if (VCP_get_char(&theByte))
		{
			int count = 0;

			// Firstly set the AudioRecBuff to the first PCM Buffer
			AudioRecBuf = PCM_Output_Buffer;


			I2S_Cmd(SPI2, ENABLE);

			j = 0;
				    while(count < 1){
				    // Press the button to stop recording, diodes will be turned on and the final three bytes sent to indicate the end
				      if(STM_EVAL_PBGetState(BUTTON_USER) == SET){
				        while(STM_EVAL_PBGetState(BUTTON_USER) == SET);
				       GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
					   // Wait some time
				        for(i=0; i<0x1000000; ++i);
					    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
				        uint8_t tab[] = {6,6,6};
				        VCP_send_buffer(tab, 3);
				        break;
				      }


				      // If the Data is ready to be copied
				      if(Data_Status){

				    	  Data_Status = 0;
				    	  // The buffers are switched in order to get the constant sound (one is being sent via COM port and one is saving the new data)
				          if (sw == 1)
				         {
				           AudioRecBuf = PCM_Output_Buffer;
				           WriteBuf = PCM_Output_Buffer1;
				           sw = 0;
				         }
				          else
				          {
				            AudioRecBuf = PCM_Output_Buffer1;
				            WriteBuf = PCM_Output_Buffer;
				            sw = 1;
				          }

				          for (i=0; i<16; i++)
				          {

				            if (buf_idx< Buffer_Input_SIZE)
				            {
				              /* Store Data in buffer input */
				              buffer_input[buf_idx++]= *(WriteBuf + i);
				              if (buf_idx1 == Buffer_Input_SIZE)
				              {
				                buf_idx1 = 0;
				                /* Send the stored data in the buffer via COM port  */
				                VCP_send_buffer(buffer_input1, 2048*2);

				                GPIOD->BSRRL = GPIO_Pin_12;
				                downTicker = 10;

				              }
				            }
				            else if (buf_idx1< Buffer_Input_SIZE)
				            {
				              /* Store Data in buffer input */
				              buffer_input1[buf_idx1++]= *(WriteBuf + i);
				              if (buf_idx == Buffer_Input_SIZE)
				              {
				                buf_idx = 0;
				                /* Send the stored data in the buffer via COM port  */
				                VCP_send_buffer(buffer_input, 2048*2);

				                GPIOD->BSRRL = GPIO_Pin_12;
				                downTicker = 10;
				              }
				            }
				          }


				          // ************************************************************
				          // YOU CAN UNCOMMENT THIS TO PERFORM FFT. You need to create:
				          // buffer_output array, which is 2x size of buffer_input
				          // buffer_output_mag array, which is 2x size of buffer_input
				          //
				          // Calculate Real FFT
				          //arm_rfft_f32(&S, buffer_input, buffer_output);
				          // Calculate magnitude
				          //arm_cmplx_mag_f32(buffer_output, buffer_output_mag, 2048);
				          // Get maximum value of magnitude
				          //arm_max_f32(&(buffer_output_mag[1]), 2048, &maxvalue, &maxvalueindex);


				        }

				      }

				    }


				    I2S_Cmd(SPI2, DISABLE);


		}
		if (0 == downTicker)
		{
			GPIOD->BSRRH = GPIO_Pin_12;
		}
		//	}

	return 0;
}



/* Private functions ---------------------------------------------------------*/
static void GPIO_Configure(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure2;
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure2);


	  // Configure MP45DT02's CLK / I2S2_CLK (PB10) line
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_25MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  // Configure MP45DT02's DOUT / I2S2_DATA (PC3) line
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_25MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);  // Connect pin 10 of port B to the SPI2 peripheral
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);   // Connect pin 3 of port C to the SPI2 peripheral


}


static void I2S_Configure(void){
  I2S_InitTypeDef I2S_InitStructure;

  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = 32000; // Audio is recorded from two channels, so this value must be twice the value in the PDM filter
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  I2S_Init(SPI2, &I2S_InitStructure);

  // Enable the Rx buffer not empty interrupt
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

static void NVIC_Configure(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	// Configure the interrupt priority grouping
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	// Configure the SPI2 interrupt channel
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

static void RCC_Configure(void)
{
	// Enable CRC module - required by PDM Library
	//RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

	/********/
	/* AHB1 */
	/********/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
		RCC_AHB1Periph_CRC, ENABLE);

	/********/
	/* APB1 */
	/********/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	RCC_PLLI2SCmd(ENABLE);

	//USB
	/* Always remember to turn on the peripheral clock...  If not, you may be up till 3am debugging... */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Setup SysTick or CROD! */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		//ColorfulRingOfDeath();
	}

	/* Setup USB */
		USBD_Init(&USB_OTG_dev,
		            USB_OTG_FS_CORE_ID,
		            &USR_desc,
		            &USBD_CDC_cb,
		            &USR_cb);
}

void SysTick_Handler(void)
{
	ticker++;
	if (downTicker > 0)
	{
		downTicker--;
	}
}

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}


