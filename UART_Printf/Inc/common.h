/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/Common.h 
  * @author  Francois HAO
  * @brief   This example shows how to retarget the C library printf function 
  *          to the Common Useage.
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
#ifndef _COMMON_H_
#define _COMMON_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern UART_HandleTypeDef UartHandle;


/*Timer GPIO definition -----------------------------------------------------*/

#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()

/* Definition for TIMx Channel Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE();
#define TIMx_GPIO_PORT_CHANNEL1        GPIOB
#define TIMx_GPIO_PORT_CHANNEL2        GPIOB
#define TIMx_GPIO_PIN_CHANNEL1         GPIO_PIN_4
#define TIMx_GPIO_PIN_CHANNEL2         GPIO_PIN_5
#define TIMx_GPIO_AF_CHANNEL1          GPIO_AF2_TIM3
#define TIMx_GPIO_AF_CHANNEL2          GPIO_AF2_TIM3


/*SPI GPIO definition -----------------------------------------------------*/

#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))



/*I2C GPIO definition -----------------------------------------------------*/



/*Button event wrapper*/
enum BTStatus{
PRESSED = 0,
NOT_PRESSED,
MAX
};



void User_Button_Init(void);
int User_Button_Status(void);
int Job_Polling_Entry(void);
void my_printf(const char *fmt, ...);
int User_UART_Init(void);
void User_PWM_Init(void);
void User_PWM_Enable(void);
int User_SPI1_Init(void);
int SPI1_Test_Application(void);





#endif

