/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f4xx_it.h"

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers                   */
/******************************************************************************/

void NMI_Handler(void) { while (1) { } }
void HardFault_Handler(void) { while (1) { } }
void MemManage_Handler(void) { while (1) { } }
void BusFault_Handler(void) { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void) { }
void DebugMon_Handler(void) { }
void PendSV_Handler(void) { }
void SysTick_Handler(void) { HAL_IncTick(); }

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  * Includes:
  * - PA15 (LMT01 Temperature sensor)
  * - PB13 (Keypad COL2)
  * - PA12 (Keypad COL3)
  */
void EXTI15_10_IRQHandler(void)
{
  /* Check line 15 (PA15 - Temperature sensor) */
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  }

  /* Check line 13 (PB13 - Keypad COL2) */
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  }

  /* Check line 12 (PA12 - Keypad COL3) */
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  }
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  * Includes:
  * - PC8 (Keypad COL1) - CRITICAL FOR KEYPAD OPERATION
  */
void EXTI9_5_IRQHandler(void)
{
  /* Check line 8 (PC8 - Keypad COL1) */
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  }}
