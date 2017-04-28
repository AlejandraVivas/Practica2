/*
 * UART.h
 *
 *  Created on: Feb 27, 2017
 *      Author: joab
 */

#ifndef SOURCE_UART_H_
#define SOURCE_UART_H_

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"
#include "pin_mux.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART_BAUDRATE0 115200U
#define UART_BAUDRATE3 115200U
#define ECHO_BUFFER_LENGTH 1
#define DEMO_UART0 UART0
#define DEMO_UART3 UART3
#define DEMO_UART0_CLKSRC UART0_CLK_SRC
#define DEMO_UART3_CLKSRC UART3_CLK_SRC
#define DEMO_UART0_RX_TX_IRQn UART0_RX_TX_IRQn
#define DEMO_UART3_RX_TX_IRQn UART3_RX_TX_IRQn
#define DEMO_UART0_IRQHandler UART0_RX_TX_IRQHandler
#define DEMO_UART3_IRQHandler UART3_RX_TX_IRQHandler


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//void uartInit_task(void *parameter);
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

void uart_init(void);
//void uart3_init(void);


#endif /* SOURCE_UART_H_ */
