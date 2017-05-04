/*
 * SerialMenu.h
 *
 *  Created on: 19/03/2017
 *      Author: Alejandra Meza
 */

#ifndef SOURCE_SERIALMENU_H_
#define SOURCE_SERIALMENU_H_

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "UART.h"
#include "GlobalFunctions.h"
#include "LCDNokia5110.h"
#include "I2C.h"

#define ESCTERA 0x1B
#define UART0FLAG 1
#define UART3FLAG 2

#define OPTION_1 0x31
#define OPTION_2 0x32
#define OPTION_3 0x33
#define OPTION_4 0x34
#define OPTION_5 0x35
#define OPTION_6 0x36
#define OPTION_7 0x37
#define OPTION_8 0x38
#define OPTION_9 0x39


/**Events for menu options*/
#define MENU_OP1 (1 << 0)
#define MENU_OP2 (1 << 1)
#define MENU_OP3 (1 << 2)
#define MENU_OP4 (1 << 3)
#define MENU_OP5 (1 << 4)
#define MENU_OP6 (1 << 5)
#define MENU_OP7 (1 << 6)
#define MENU_OP8 (1 << 7)
#define MENU_OP9 (1 << 8) //SPI

#define ENTERTERA 0xD
#define ENTERBLUE 0x5B
#define ESCTERA 0x1B
#define ESCBLUE 0x5D


#define RTC_DEVICE_ADD 0x6F
#define MEM_DEVICE_ADD 0x50

#define ASCII_NUMBER_MASK 0x30	//Mask used to add or substract to an HEX number value to convert it from/to ASCII value
#define ASCII_LETTER_MASK 0x37	//Mask used to add or substract to an HEX letter value to convert it from/to ASCII value
#define BCD_H 0xF0				//Mask used to make a bitwise operation with the high part of the BCD data
#define BCD_L 0x0F				//Mask used to make a bitwise operation with the low part of the BCD data
#define SECONDS_REG_SIZE 0x7F	//Used to set reading boundaries according to the seconds register size
#define MINUTES_REG_SIZE 0x7F	//Used to set reading boundaries according to the minutes register size
//#define HOURS_REG_SIZE 0x1F		//Used to set reading boundaries according to the hours register size
#define DAY_REG_SIZE 0x3F		//Used to set reading boundaries according to the days register size
#define MONTH_REG_SIZE 0x1F		//Used to set reading boundaries according to the month register size
#define YEAR_REG_SIZE 0xFF		//Used to set reading boundaries according to the year register size

#define ASCII_NUMBER_MASK 0x30	//Mask used to add or substract to an HEX number value to convert it from/to ASCII value
#define ASCII_LETTER_MASK 0x37	//Mask used to add or substract to an HEX letter value to convert it from/to ASCII value


void PORTC_IRQHandler(void);
void createSemaphoreMutex(void);

/*Uart 0*/
void readingI2C_task(void *arg);
void writingI2C_task(void *arg);
void setHour_task(void *arg);
void setDate_task(void *arg);
void hourFormat_task(void *arg);
void readHour_task(void *pvParameters);
void readDate_task(void *arg);
void chat_task(void *arg);
void eco_task(void *arg);
void Client_task(void *pvParameters);

void getTime_task(void *pvParameters);
void serialTimeLCD(void *pvParameters);

uint16_t asciiToHex(uint8_t *string);
uint16_t asciiToDec(uint8_t *string);

//static void printingReadMemMenu(UART_Type *base);
//static void printingWriteMemMenu(UART_Type *base);
//static void printingSetHourMenu(UART_Type *base);
//static void printingSetDateMenu(UART_Type *base);
//static void printingSetFormatMenu(UART_Type *base);
//static void printingReadHourMenu(UART_Type *base);
//static void printingReadDateMenu(UART_Type *base);
//static void printingChatMenu(UART_Type *base);
//static void printingEcoMenu(UART_Type *base);



#endif /* SOURCE_SERIALMENU_H_ */
