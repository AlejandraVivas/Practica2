/*
 * UART.c
 *
 *  Created on: Feb 27, 2017
 *      Autor: Alejandra Meza
 */


#include "FreeRTOS.h"
#include "UART.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_uart.h"
#include "MK64F12.h"
#include "semphr.h"
#include "pin_mux.h"
#include "clock_config.h"

void uart_init(void)
{

    uart_config_t config;

    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = UART_BAUDRATE0;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART0, &config, CLOCK_GetFreq(DEMO_UART0_CLKSRC));

    uart_config_t config3;

    UART_GetDefaultConfig(&config3);
    config3.baudRate_Bps = UART_BAUDRATE3;
    config3.enableTx = true;
    config3.enableRx = true;

    UART_Init(DEMO_UART3, &config3, CLOCK_GetFreq(DEMO_UART3_CLKSRC));

	UART_EnableInterrupts(DEMO_UART0, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
	EnableIRQ(DEMO_UART0_RX_TX_IRQn);
	UART_EnableInterrupts(DEMO_UART3, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
	EnableIRQ(DEMO_UART3_RX_TX_IRQn);

}

