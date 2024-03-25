/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/Common.c 
  * @author  Francois HAO
  * @brief   This example shows how to retarget the C library printf function 
  *          to the Common useage.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 University of Warwick.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
/* Private typedef -----------------------------------------------------------*/
#define  PERIOD_VALUE       (uint32_t)(1500 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*12.5/100) /* Capture Compare 4 Value  */

/* SPI macro */
#define MASTER_BOARD
//#define SLAVE_BOARD


/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;


/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

UART_HandleTypeDef UartHandle;


/* Private variables ---------------------------------------------------------*/


/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Interrupt **** SPI Message ******** SPI Message ******** SPI Message ****";

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];



static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
  }
}


void Test_Application(void)
{
	if (User_Button_Status() == PRESSED)
	{
		my_printf("\r\n  test button has not been pressed \r\n");
		//BSP_LED_On(LED2);
		BSP_LED_Toggle(LED2);
	}
	if (User_Button_Status() == NOT_PRESSED)
	{
		my_printf("\r\n  test button has not been pressed \r\n");
		BSP_LED_Off(LED2);
	}
}


void User_PWM_Init(void)
{
	
	/* Compute the prescaler value to have TIM3 counter clock equal to 15000000 Hz */
	uhPrescalerValue = (uint32_t)((SystemCoreClock/2) / 15000000) - 1;
	
	/*##-1- Configure the TIM peripheral #######################################*/
	/* -----------------------------------------------------------------------
	TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.
	
	  In this example TIM3 input clock (TIM3CLK) is set to APB1 clock x 2,
	  since APB1 prescaler is equal to 2.
		TIM3CLK = APB1CLK*2
		APB1CLK = HCLK/2
		=> TIM3CLK = HCLK = SystemCoreClock
	
	  To get TIM3 counter clock at 15 MHz, the prescaler is computed as follows:
		 Prescaler = (TIM3CLK / TIM3 counter clock) - 1
		 Prescaler = ((SystemCoreClock) /15 MHz) - 1
	
	  To get TIM3 output clock at 22,52 KHz, the period (ARR)) is computed as follows:
		 ARR = (TIM3 counter clock / TIM3 output clock) - 1
			 = 665
	
	  TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR + 1)* 100 = 50%
	  TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR + 1)* 100 = 37.5%
	  TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR + 1)* 100 = 25%
	  TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR + 1)* 100 = 12.5%
	
	  Note:
	   SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	   Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	   variable value. Otherwise, any configuration based on this variable will be incorrect.
	   This variable is updated in three ways:
		1) by calling CMSIS function SystemCoreClockUpdate()
		2) by calling HAL API function HAL_RCC_GetSysClockFreq()
		3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
	----------------------------------------------------------------------- */
	
	/* Initialize TIMx peripheral as follows:
		 + Prescaler = (SystemCoreClock / 15000000) - 1
		 + Period = (666 - 1)
		 + ClockDivision = 0
		 + Counter direction = Up
	*/
	TimHandle.Instance = TIMx;
	
	TimHandle.Init.Prescaler		 = uhPrescalerValue;
	TimHandle.Init.Period			 = PERIOD_VALUE;
	TimHandle.Init.ClockDivision	 = 0;
	TimHandle.Init.CounterMode		 = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
	{
	  /* Initialization Error */
	  Error_Handler();
	}
	
	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration for all channels */
	sConfig.OCMode		 = TIM_OCMODE_PWM1;
	sConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	
	/* Set the pulse value for channel 1 */
	sConfig.Pulse = PULSE1_VALUE;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
	  /* Configuration Error */
	  Error_Handler();
	}
	
	/* Set the pulse value for channel 2 */
	sConfig.Pulse = PULSE2_VALUE;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
	{
	  /* Configuration Error */
	  Error_Handler();
	}
	
}


void User_PWM_Enable(void)
{
	/*##-3- Start PWM signals generation #######################################*/
	/* Start channel 1 */
	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
	{
	  /* PWM Generation Error */
	  Error_Handler();
	}
	/* Start channel 2 */
	if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
	{
	  /* PWM Generation Error */
	  Error_Handler();
	}
}


void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&UartHandle, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

inline void my_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}


int User_UART_Init(void)
{
    int ret = HAL_OK;
	UartHandle.Instance 		 = USARTx;
	
	UartHandle.Init.BaudRate	 = 9600;
	UartHandle.Init.WordLength	 = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits	 = UART_STOPBITS_1;
	UartHandle.Init.Parity		 = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl	 = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode		 = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	  
	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
	  /* Initialization Error */
	    ret = HAL_ERROR;
	    //Error_Handler(); 
	}
    return ret;
}

void User_Button_Init(void)
{
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
}

int User_SPI1_Init(void)
{
    int ret = HAL_OK;
	  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
	  SpiHandle.Instance               = SPIx;
	  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
	  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	  SpiHandle.Init.CRCPolynomial     = 7;
	  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	  
#ifdef MASTER_BOARD
	  SpiHandle.Init.Mode = SPI_MODE_MASTER;
#else
	  SpiHandle.Init.Mode = SPI_MODE_SLAVE;
#endif /* MASTER_BOARD */

	  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    //Error_Handler();
	    ret = HAL_ERROR;
      }
	return ret;
}

int User_Button_Status(void)
{
    /*Button has not been pressed, gpio 3.3V*/
    int ret = NOT_PRESSED;
    if (BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_RESET)
        ret = NOT_PRESSED;
    if (BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_SET)
        ret = PRESSED;
    return ret;
}

static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

static void Timeout_Error_Handler(void)
{
  /* Toggle LED4 on */
  while(1)
  {
    HAL_Delay(500);
  }
}


int SPI1_Test_Application(void)
{
    int ret = HAL_OK;
	while(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE,5000) != HAL_OK)
	{
	  /* Transfer error in transmission process */
	  // Error_Handler();
	  //memset(aRxBuffer, 0xFF, sizeof(aRxBuffer));
	  ret = HAL_ERROR;
	}
	
	/*##-3- Wait for the end of the transfer ###################################*/	
	/*	Before starting a new communication transfer, you need to check the current   
		state of the peripheral; if it�s busy you need to wait for the end of current
		transfer before starting a new one.
		For simplicity reasons, this example is just waiting till the end of the 
		transfer, but application may perform other tasks while transfer operation
		is ongoing. */	
	while (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_READY)
	{
	} 
	
	/*##-4- Compare the sent and received buffers ##############################*/
	if(Buffercmp((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, BUFFERSIZE))
	{
	  /* Transfer error in transmission process */
	  // Error_Handler();
	  //memset(aRxBuffer, 0, sizeof(aRxBuffer));
	  ret = HAL_ERROR;
	}


	return ret;

}




void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED3 on: Transfer process is correct */
  //BSP_LED_On(LED2);
  if (hspi == &SpiHandle)
	{
		HAL_SPI_TransmitReceive_IT(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE);
	}
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED4 on: Transfer error in reception/transmission process */
  //BSP_LED_Off(LED2); 
}



int Job_Polling_Entry(void)
{
    /*There are many type of periods tasks like 1ms, 5ms, 10ms, 20ms, 50ms, 100ms.*/
    static unsigned int counter;
	int ret = 1;
	counter ++;
	if (counter % 5 == 0)
	{
	    /*5ms task*/
	    
	}
	if (counter % 10 == 0)
	{
	    /*10ms task*/
	Test_Application();
        
	}
	if (counter % 20 == 0)
	{
	    /*20ms task*/
	}
	if (counter % 99 == 0)
	{
	    /*99ms task*/
	    //SPI1_Test_Application();
	}
	if (counter >= 100)
		counter = 0;
	HAL_Delay(1);
	return ret;	
}




