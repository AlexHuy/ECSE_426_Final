#ifndef _UART_H_
#define _UART_H_

#define USE_STM32F4XX_NUCLEO

#include "cube_hal.h"

void MX_USART1_UART_Init(void);
void USART1_Receive(uint8_t *pData, uint16_t Size);
void USART1_Transmit(uint8_t *pData, uint16_t Size);

#endif