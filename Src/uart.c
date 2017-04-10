#include "uart.h"
#include "supporting_functions.h"

UART_HandleTypeDef huart1;
GPIO_InitTypeDef gpioAInit;

void MX_USART1_UART_Init(void)
{
	//usart1 transmitting pin
	gpioAInit.Pin = GPIO_PIN_9|GPIO_PIN_10;
	gpioAInit.Mode = GPIO_MODE_AF_PP;
	gpioAInit.Pull = GPIO_PULLUP;
  gpioAInit.Speed = GPIO_SPEED_FREQ_HIGH;
	gpioAInit.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA,&gpioAInit);
	
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler(1);
  }
	
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0); //When the NVIC_PriorityGroup_0 is selected, IRQ preemption is no more possible.
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0); // first 0 = preemptive priority, second 0 = sub priority
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_Receive(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Receive_IT(&huart1, pData, Size);
}

void USART1_Transmit(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit_IT(&huart1, pData, Size);
}