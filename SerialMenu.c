/*
 * SerialMenu.c
 *
 *  Created on: 19/03/2017
 *      Author: Alejandra Meza
 */

#include "SerialMenu.h"
#include "err.h"
#include "lwip/netbuf.h"
#include "lwip/memp.h"
#include "lwip/api.h"
#include "myFiles/LCDNokia5110.h"



uint8_t menuString[] = "Menu\r\n";
uint8_t optionString[] = "Opciones:\r\n";
uint8_t readMemoryString[] = "1) Leer Memoria\r\n";
uint8_t writeMemoryString[] = "2) Escribir Memoria\r\n";
uint8_t setHourString[] = "3) Establecer Hora\r\n";
uint8_t setDateString[] = "4) Establecer Fecha\r\n";
uint8_t hourFormatString[] = "5) Formato de Hora\r\n";
uint8_t readHourString[] = "6) Leer Hora\r\n";
uint8_t readDateString[] = "7) Leer Fecha\r\n";
uint8_t chatString[] = "8) Comunicacion con terminal 2\r\n";
uint8_t ecoString[] = "9) Eco en LCD\r\n";
uint8_t NewLineString[] = "\n";


/*Read Memory Menu*/
uint8_t memoryDirectionString[] = "Direccion de Lectura:\n";
uint8_t lenghtToReadString[] = "Longitud en bytes:\n";
uint8_t memoryDirectionString2[] = "Direccion de Lectura: 0x\r";
uint8_t lenghtToReadString2[] = "Longitud en bytes:\r";
uint8_t contentString[] = "\nContenido:\r\n";

/*Write Memory Menu*/
uint8_t memoryWriteDirectionString[] = "Direccion de Escritura:\r\n";
uint8_t lenghtToWriteString[] = "Longitud en bytes:\r\n";
uint8_t savedTextString[] = "Texto a guardar:\r\n";
uint8_t savedString[] = "Su texto ha sido guardado\r\n";

/*Set hour Menu*/
uint8_t setNewHourString[] = "Escribir hora en formato hh/mm/ss:\r\n";
uint8_t savedHourString[] = "La hora ha sido cambiada\r\n";

/*Set Date Menu*/
uint8_t setNewDateString[] = "Escribir fecha en formato dd/mm/aa:\r\n";
uint8_t savedDateString[] = "La fecha ha sido cambiada\r\n";

/*Set Format Menu*/
uint8_t setNewFormatString[] = "Formato cambiado\r\n";
uint8_t setNewFormatOptionString[] = "Para Formato 12 hrs presione 1, para formato de 24 hrs presiona 2: \r\n";

/*Read Hour Menu*/
uint8_t readActualHourString[] = "La hora actual es:\r";

/*Read Date Menu*/
uint8_t readActualDateString[] = "La fecha actual es:\r";

/*Chat Menu*/
uint8_t showingChatString[] = "Chat:\r";
uint8_t TerminalOneEndingString[] = "Terminal se ha desconectado";

/*Eco Menu*/
uint8_t ecoLcdString[] = "Eco en pantalla LCD\r";

uint8_t keyPressedNotvalidString[] = "Tecla no valida, presiona un numero entre 0 y 9\r\n";

uint8_t pointsString[] = ":";
uint8_t lineString[] = "-";
uint8_t twentyString[] = "-20";

uint8_t errorString[] = "error";
uint8_t HoraString[] = "Hora: ";
uint8_t fechaString[] = "Fecha: ";
uint8_t twentyCeroString[] = "-20";

uint8_t exitString[] = "\nPresione e para salir \n\r";

uint8_t adjust1[] = "\033[1;1H";
uint8_t adjust2[] = "\033[2;1H";
uint8_t adjust3[] = "\033[1;25H";
uint8_t adjust4[] = "\033[3;1H";
uint8_t adjust5[] = "\033[2;23H";
uint8_t adjust6[] = "\033[1;32H";
uint8_t adjust7[] = "\033[2;33H";
uint8_t adjust8[] = "\033[2;30H";
uint8_t adjust9[] = "\033[1;33H";
uint8_t adjut10[] = "\033[12;1H";
uint8_t adjust11[] = "\033[4;1H";
uint8_t adjust12[] = "\033[3;18H";
uint8_t deleteLine[] = "\033[2K";

uint8_t readHourUart0Flag;
uint8_t readHourUart3Flag;

uint8_t readDateUart0Flag;
uint8_t readDateUart3Flag;


TaskHandle_t readingI2CHandle = NULL;
TaskHandle_t writingI2CHandle = NULL;
TaskHandle_t setHourHandle = NULL;
TaskHandle_t setDateHandle = NULL;
TaskHandle_t hourFormatHandle = NULL;
TaskHandle_t readHourHandle = NULL;
TaskHandle_t readDateHandle = NULL;
TaskHandle_t chatHandle = NULL;
TaskHandle_t ecoHandle = NULL;
TaskHandle_t setNewDateHandle = NULL;

extern TaskHandle_t lcdHandle;
extern TaskHandle_t client_task_handler;


/*get time*/
uint16_t hexAddress;
uint8_t counter = 0;
uint8_t timeBuffer[7];
uint8_t qCounter=0;
uint8_t asciiDate[12];
uint8_t *ptrToDate = asciiDate;
uint8_t HOURS_REG_SIZE = 0x1F;
uint8_t myRtcDataTime[5];
uint8_t myRtcDataDate[5];
/****/

xSemaphoreHandle i2cProtected = FALSE;

extern err_t err;




//void PORTC_IRQHandler(void)
//{
//	bitNumber = GPIO_GetPinsInterruptFlags(GPIOC);
//	switch(bitNumber)
//	{
//	case BIT5:
//		GPIO_ClearPinsInterruptFlags(GPIOC, BIT5);
//		buttonOneFlag = TRUE;
//		break;
//	case BIT7:
//		GPIO_ClearPinsInterruptFlags(GPIOC, BIT7);
//		buttonTwoFlag = TRUE;
//		break;
//	case BIT0:
//		GPIO_ClearPinsInterruptFlags(GPIOC, BIT0);
//		buttonThreeFlag = TRUE;
//		break;
//	case BIT9:
//		GPIO_ClearPinsInterruptFlags(GPIOC, BIT9);
//		buttonFourFlag = TRUE;
//		break;
//	default:
//		break;
//	}
//	xSemaphoreGiveFromISR( xCountingSemaphore, &xHigherPriorityTaskWoken );
//	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//}


void printingMenu(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);

	i2cProtected = xSemaphoreCreateMutex();

	netconn_write(newconn,menuString,sizeof(menuString), 0x01);
	netconn_write(newconn,optionString,sizeof(optionString), 0x01);
	netconn_write(newconn,readMemoryString,sizeof(readMemoryString), 0x01);
	netconn_write(newconn,writeMemoryString,sizeof(writeMemoryString), 0x01);
	netconn_write(newconn,setHourString,sizeof(setHourString), 0x01);
	netconn_write(newconn,setDateString,sizeof(setDateString), 0x01);
	netconn_write(newconn,hourFormatString,sizeof(hourFormatString), 0x01);
	netconn_write(newconn,readHourString,sizeof(readHourString), 0x01);
	netconn_write(newconn,readDateString,sizeof(readDateString), 0x01);
	netconn_write(newconn,chatString,sizeof(chatString), 0x01);
	netconn_write(newconn,ecoString,sizeof(ecoString), 0x01);
	menuOptionSelection((void *)newconn);
}

void menuOptionSelection(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	uint8_t option;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	//err_t err;
	err = netconn_recv(newconn, &buf);
	netbuf_data(buf, &data, &len);
	option = *(char*)data;
	netbuf_delete(buf);
	switch(option)
	{
	case '1':
		readingI2C_task((void *)newconn);
		break;
	case '2':
		writingI2C_task((void *)newconn);
		break;
	case '3':
		setHour_task((void *)newconn);
		break;
	case '4':
		setDate_task((void *)newconn);
		break;
	case '5':
		hourFormat_task((void *)newconn);
		break;
	case '6':
		readHour_task((void *)newconn);
		break;
	case '7':
		break;
	case '8':
		break;
	case '9':
		LCDNokia_clear();
		eco_task((void *)newconn);
		break;
	default:
		break;
	}
}

void readingI2C_task(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	uint8_t option;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	static uint16_t readAddress;
	static uint32_t readLenght;
	static uint8_t testAddress[4];
	uint8_t memoryCounter = 0;
	uint8_t lengthToRead[3];
	static uint8_t midatoMemoria[7]; /**address that's going to be read*/
	uint8_t readValue[255];
	uint8_t key_pressedMemory = TRUE;
	for(;;)
	{
		if(TRUE == key_pressedMemory)
		{
			netconn_write(newconn, memoryDirectionString, sizeof(memoryDirectionString), NETCONN_COPY);
			while((err = netconn_recv(newconn, &buf)) == ERR_OK)
			{
				netbuf_data(buf, &data, &len);
				option = *(char*)data;
				if('e' != option)
				{
					midatoMemoria[memoryCounter] = option;
					memoryCounter++;

					if(memoryCounter == 4)
					{
						testAddress[0] = midatoMemoria[0];
						testAddress[1] = midatoMemoria[1];
						testAddress[2] = midatoMemoria[2];
						testAddress[3] = midatoMemoria[3];

						delay(65000);

						netconn_write(newconn, lenghtToReadString, sizeof(lenghtToReadString), NETCONN_COPY);

					}
					else if(memoryCounter == 7)
					{
						lengthToRead[0] = midatoMemoria[4];
						lengthToRead[1] = midatoMemoria[5];
						lengthToRead[2] = midatoMemoria[6];
					}

					if('*' == option)
					{
						key_pressedMemory = FALSE;
						readAddress = asciiToHex(testAddress);
						readLenght = asciiToHex(lengthToRead);
						netconn_write(newconn, memoryDirectionString2, sizeof(memoryDirectionString2), NETCONN_COPY);
						netconn_write(newconn, testAddress, sizeof(testAddress), NETCONN_COPY);
						netconn_write(newconn, NewLineString, sizeof(NewLineString), NETCONN_COPY);
						netconn_write(newconn, lenghtToReadString2, sizeof(lenghtToReadString2), NETCONN_COPY);
						netconn_write(newconn, lengthToRead, sizeof(lengthToRead), NETCONN_COPY);
						for(;;)
						{
							if(xSemaphoreTake(i2cProtected,1000))
							{
								I2C_MemoryRead(I2C0, MEM_DEVICE_ADD,readAddress,readValue, readLenght);
								xSemaphoreGive(i2cProtected);
								vTaskDelay(100);
								break;
							}
						}
						netconn_write(newconn,contentString , sizeof(contentString), NETCONN_COPY);
						netconn_write(newconn, readValue, readLenght, NETCONN_COPY);
						netconn_write(newconn, exitString, sizeof(exitString), NETCONN_COPY);
					}

				}
				else if('e' == option)
				{
					printingMenu((void *)newconn);
				}
				while (netbuf_next(buf) >= 0);
				netbuf_delete(buf);
			}
		}
	}
}

void writingI2C_task(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	uint8_t option;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	static uint16_t readAddress;
	static uint32_t readLenght;
	uint8_t dataWrited[200];
	uint8_t dataIn[7];
	static uint8_t dataLong[3];
	static uint8_t writeDirection[4];
	uint8_t counter = 0;
	uint8_t dataCounter = 0;
	uint8_t key_pressedMemory = TRUE;
	uint8_t key_pressedDataIn = FALSE;
	for(;;)
	{
		if(TRUE == key_pressedMemory)
		{
			netconn_write(newconn, memoryWriteDirectionString, sizeof(memoryWriteDirectionString), NETCONN_COPY);
			while((err = netconn_recv(newconn, &buf)) == ERR_OK)
			{
				netbuf_data(buf, &data, &len);
				option = *(char*)data;
				dataIn[counter] = option;
				counter++;
				if(TRUE == key_pressedDataIn)
				{
					dataWrited[dataCounter] = option;
					dataCounter++;
					if('e' == option){
						netconn_write(newconn, savedTextString, sizeof(savedTextString), NETCONN_COPY);
						for(;;)
						{
							if(xSemaphoreTake(i2cProtected,1000))
							{
								I2C_MemoryWrite(I2C0, MEM_DEVICE_ADD, readAddress, dataWrited, readLenght);
								xSemaphoreGive(i2cProtected);
								vTaskDelay(100);
								break;
							}
						}
						netconn_write(newconn, savedString,sizeof(savedString) , NETCONN_COPY);
						netconn_write(newconn, "Presione q para salir\r\n",25 , NETCONN_COPY);
					}

					if('q' == option)
					{
						printingMenu((void *)newconn);
					}
				}


				else if(counter == 4)
				{
					writeDirection[0] = dataIn[0];
					writeDirection[1] = dataIn[1];
					writeDirection[2] = dataIn[2];
					writeDirection[3] = dataIn[3];

					delay(65000);
					readAddress = asciiToHex(writeDirection);
					netconn_write(newconn, lenghtToReadString, sizeof(lenghtToReadString), NETCONN_COPY);

				}
				else if(counter == 7)
				{
					dataLong[0] = dataIn[4];
					dataLong[1] = dataIn[5];
					dataLong[2] = dataIn[6];
				}

				if('*' == option)
				{
					readLenght =  asciiToHex(dataLong);
					key_pressedMemory = FALSE;
					key_pressedDataIn = TRUE;
				}
				while (netbuf_next(buf) >= 0);
				netbuf_delete(buf);
			}
		}
	}
}

void setHour_task(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	uint8_t option;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	static uint8_t counter = 0;
	netconn_write(newconn, setNewHourString, sizeof(setNewHourString), NETCONN_COPY);
	for(;;)
	{
		while((err = netconn_recv(newconn, &buf)) == ERR_OK)
		{
			netbuf_data(buf, &data, &len);
			option = *(char*)data;

			if('e' != option)
			{
				myRtcDataTime[counter] = option;
				counter++;
			}
			else if('e' == option)
			{
				counter = 0;

				for(;;)
				{
					if(xSemaphoreTake(i2cProtected,1000))
					{
						I2C_RtcWrite(I2C0, RTC_DEVICE_ADD, 0x02, ((myRtcDataTime[0]-ASCII_NUMBER_MASK))<<4 | (myRtcDataTime[1]-ASCII_NUMBER_MASK));
						delay(1000);
						I2C_RtcWrite(I2C0, RTC_DEVICE_ADD, 0x01, ((myRtcDataTime[2]-ASCII_NUMBER_MASK))<<4 | (myRtcDataTime[3]-ASCII_NUMBER_MASK));
						delay(1000);
						I2C_RtcWrite(I2C0, RTC_DEVICE_ADD,0x00, (0x80 | ((myRtcDataTime[4]-ASCII_NUMBER_MASK))<<4 | (myRtcDataTime[5]-ASCII_NUMBER_MASK)));
						xSemaphoreGive(i2cProtected);
						vTaskDelay(100);
						break;
					}
				}
				netconn_write(newconn, savedHourString, sizeof(savedHourString), NETCONN_COPY);
				netconn_write(newconn, "Presione q para salir\r\n",25 , NETCONN_COPY);
			}
			if('q' == option)
			{
				printingMenu((void *)newconn);
			}
			while (netbuf_next(buf) >= 0);
			netbuf_delete(buf);
		}
	}
}

void setDate_task(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	uint8_t option;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	static uint8_t counter = 0;
	netconn_write(newconn, setNewDateString, sizeof(setNewDateString), NETCONN_COPY);
	for(;;)
	{
		while((err = netconn_recv(newconn, &buf)) == ERR_OK)
		{
			netbuf_data(buf, &data, &len);
			option = *(char*)data;
			if('e' != option)
			{
				myRtcDataDate[counter] = option;
				counter++;
			}
			else if('e' == option)
			{
				counter = 0;
				for(;;)
				{
					if(xSemaphoreTake(i2cProtected,1000))
					{
						I2C_RtcWrite(I2C0, RTC_DEVICE_ADD, 0x04, ((myRtcDataDate[0]-ASCII_NUMBER_MASK))<<4 | (myRtcDataDate[1]-ASCII_NUMBER_MASK));
						delay(1000);
						I2C_RtcWrite(I2C0, RTC_DEVICE_ADD, 0x05, ((myRtcDataDate[2]-ASCII_NUMBER_MASK))<<4 | (myRtcDataDate[3]-ASCII_NUMBER_MASK));
						delay(1000);
						I2C_RtcWrite(I2C0, RTC_DEVICE_ADD,0x06, ((myRtcDataDate[4]-ASCII_NUMBER_MASK))<<4 | (myRtcDataDate[5]-ASCII_NUMBER_MASK));
						xSemaphoreGive(i2cProtected);
						vTaskDelay(100);
						break;
					}
				}
				netconn_write(newconn, savedDateString, sizeof(savedDateString), NETCONN_COPY);
				netconn_write(newconn, "Presione q para salir\r\n",25 , NETCONN_COPY);
			}
			if('q' == option)
			{
				printingMenu((void *)newconn);
			}
			while (netbuf_next(buf) >= 0);
			netbuf_delete(buf);
		}
	}
}

void hourFormat_task(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	uint8_t option;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	uint8_t twelveHoursFormat = 0x40;
	uint8_t twentyFourHoursFormat = 0x00;
	netconn_write(newconn, setNewFormatOptionString, sizeof(setNewFormatOptionString), NETCONN_COPY);
	for(;;)
	{
		while((err = netconn_recv(newconn, &buf)) == ERR_OK)
		{
			netbuf_data(buf, &data, &len);
			option = *(char*)data;
			if('e' != option)
			{
				if('1' == option)
				{
					HOURS_REG_SIZE = (twelveHoursFormat &  (asciiDate[5] | asciiDate[4]) );
					for(;;)
					{
						if(xSemaphoreTake(i2cProtected,1000))
						{
							I2C_RtcWrite(I2C0, RTC_DEVICE_ADD, 0x02,HOURS_REG_SIZE);
							delay(1000);
							xSemaphoreGive(i2cProtected);
							vTaskDelay(100);
							break;
						}
					}
					netconn_write(newconn, setNewFormatString, sizeof(setNewFormatString), NETCONN_COPY);
					netconn_write(newconn, "Presione e para salir\r\n",25 , NETCONN_COPY);
				}
				else if('2' == option)
				{
					HOURS_REG_SIZE = ( twentyFourHoursFormat &  (asciiDate[5] | asciiDate[4]));
					for(;;)
					{
						if(xSemaphoreTake(i2cProtected,1000))
						{
							I2C_RtcWrite(I2C0, RTC_DEVICE_ADD, 0x02,HOURS_REG_SIZE);
							delay(1000);
							xSemaphoreGive(i2cProtected);
							vTaskDelay(100);
							break;
						}
					}
					netconn_write(newconn, setNewFormatString, sizeof(setNewFormatString), NETCONN_COPY);
					netconn_write(newconn, "Presione e para salir\r\n",25 , NETCONN_COPY);
				}
			}
			else if('e' == option)
			{
				printingMenu((void *)newconn);
			}
			while (netbuf_next(buf) >= 0);
			netbuf_delete(buf);
		}
	}
}

void readHour_task(void *arg)
{
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	uint8_t option;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	uint8_t hourD;
	uint8_t hourU;
	uint8_t minD;
	uint8_t minU;
	uint8_t secD;
	uint8_t secU;
	for(;;)
	{
		err = netconn_recv(newconn, &buf);

			netbuf_data(buf, &data, &len);
			option = *(char*)data;
			hourD = asciiDate[5];
			hourU = asciiDate[4];
			minD = asciiDate[3];
			minU = asciiDate[2];
			secD = asciiDate[1];
			secU = asciiDate[0];
			netconn_write(newconn,"\n\n\n\n\n\n\n\n\n\n\n",sizeof("\n\n\n\n\n\n\n\n\n\n\n"), NETCONN_COPY);
			netconn_write(newconn, &hourD ,sizeof(hourD), NETCONN_COPY);
			netconn_write(newconn, &hourU ,sizeof(hourU), NETCONN_COPY);
			netconn_write(newconn, &minD ,sizeof(minD), NETCONN_COPY);
			netconn_write(newconn, &minU ,sizeof(minU), NETCONN_COPY);
			netconn_write(newconn, &secD ,sizeof(secD), NETCONN_COPY);
			netconn_write(newconn, &secU ,sizeof(secU), NETCONN_COPY);
			vTaskDelay(1000);
			if('e' == option)
			{
				printingMenu((void *)newconn);
			}
	}
}

//
//void readDate_task(void *arg)
//{
//	static uint8_t received_data;
//	UART_Type *currentUart;
//	for(;;)
//	{
//		if(MENU_OP7 == xEventGroupGetBits(Event_uartHandle0) || MENU_OP7 == xEventGroupGetBits(Event_uartHandle3))
//		{
//			if(readDateUart0Flag  == 1)
//			{
//				received_data = uart0Data;
//				readDateUart0Flag = 0;
//				currentUart = DEMO_UART0;
//				if(ESCTERA != received_data)
//				{
//					UART_WriteByte(currentUart, asciiDate[7]);
//					UART_WriteByte(currentUart, asciiDate[6]);
//					UART_WriteBlocking(currentUart, lineString, sizeof(lineString) / sizeof(lineString[0]));
//					UART_WriteByte(currentUart, asciiDate[9]);
//					UART_WriteByte(currentUart, asciiDate[8]);
//					UART_WriteBlocking(currentUart, twentyString, sizeof(twentyString) / sizeof(twentyString[0]));
//					UART_WriteByte(currentUart, asciiDate[11]);
//					UART_WriteByte(currentUart, asciiDate[10]);
//					UART_WriteBlocking(currentUart, adjust2, sizeof(adjust2) / sizeof(adjust2[0]));
//					vTaskDelay(1000);
//				}
//			}
//			else if(readDateUart3Flag == 1)
//			{
//				readDateUart3Flag = 0;
//				currentUart = DEMO_UART3;
//				if(ESCTERA != received_data)
//				{
//					UART_WriteBlocking(currentUart, adjust2, sizeof(adjust2) / sizeof(adjust2[0]));
//					delay(100);
//					UART_WriteByte(currentUart, asciiDate[7]);
//					delay(100);
//					UART_WriteByte(currentUart, asciiDate[6]);
//					delay(100);
//					UART_WriteBlocking(currentUart, lineString, sizeof(lineString) / sizeof(lineString[0]));
//					delay(100);
//					UART_WriteByte(currentUart, asciiDate[9]);
//					delay(100);
//					UART_WriteByte(currentUart, asciiDate[8]);
//					delay(100);
//					UART_WriteBlocking(currentUart, twentyString, sizeof(twentyString) / sizeof(twentyString[0]));
//					delay(100);
//					UART_WriteByte(currentUart, asciiDate[11]);
//					delay(100);
//					UART_WriteByte(currentUart, asciiDate[10]);
//					delay(100);
//					UART_WriteBlocking(currentUart, adjust4, sizeof(adjust4) / sizeof(adjust4[0]));
//					vTaskDelay(1000);
//				}
//			}
//		}
//		vTaskDelay(1);
//		taskYIELD();
//	}
//}
//
//void chat_task(void *arg)
//{
//	static uint8_t received_data;
//	UART_Type *currentUart;
//	static uint8_t testBufferOne[60] = {0};
//	static uint8_t testBufferFour[60] = {0};
//	static uint8_t TerminalOneString[] = "Terminal 1:\r\n";
//	static uint8_t TerminalTwoString[] = "Terminal 2:\r\n";
//	static uint8_t bufferOneCounter = 0;
//	static uint8_t bufferFourCounter = 0;
//	for(;;)
//	{
//		if(MENU_OP8 == xEventGroupGetBits(Event_uartHandle0) && (MENU_OP8 == xEventGroupGetBits(Event_uartHandle3)))
//		{
//			if(xSemaphoreTake(NewDataUart0, portMAX_DELAY))
//			{
//				received_data = uart0Data;
//				currentUart = DEMO_UART0;
//				if(ENTERTERA != uart0Data)
//				{
//					UART_WriteByte(DEMO_UART0, uart0Data);
//					testBufferOne[bufferOneCounter] = uart0Data;
//					bufferOneCounter++;
//				}
//				else if(ENTERTERA == uart0Data)
//				{
//					bufferOneCounter = 0;
//					UART_WriteBlocking(DEMO_UART0, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//					UART_WriteBlocking(DEMO_UART0, TerminalOneString, sizeof(TerminalOneString) / sizeof(TerminalOneString[0]));
//					UART_WriteBlocking(DEMO_UART0, testBufferOne, sizeof(testBufferOne) / sizeof(testBufferOne[0]));
//					UART_WriteBlocking(DEMO_UART0, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//					UART_WriteBlocking(DEMO_UART3, TerminalOneString, sizeof(TerminalOneString) / sizeof(TerminalOneString[0]));
//					UART_WriteBlocking(DEMO_UART3, testBufferOne, sizeof(testBufferOne) / sizeof(testBufferOne[0]));
//					UART_WriteBlocking(DEMO_UART3, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//				}
//			}
//			if(xSemaphoreTake(NewDataUart3, portMAX_DELAY))
//			{
//				received_data = uart3Data;
//				currentUart = DEMO_UART3;
//				if(ENTERTERA != uart3Data)
//				{
//					UART_WriteByte(DEMO_UART3, uart3Data);
//					testBufferFour[bufferFourCounter] = uart3Data;
//					bufferFourCounter++;
//				}
//				else if(ENTERTERA == uart3Data)
//				{
//					bufferFourCounter = 0;
//					UART_WriteBlocking(DEMO_UART3, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//					UART_WriteBlocking(DEMO_UART3, TerminalTwoString, sizeof(TerminalTwoString) / sizeof(TerminalTwoString[0]));
//					UART_WriteBlocking(DEMO_UART3, testBufferFour, sizeof(testBufferFour) / sizeof(testBufferFour[0]));
//					UART_WriteBlocking(DEMO_UART3, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//					UART_WriteBlocking(DEMO_UART0, TerminalTwoString, sizeof(TerminalTwoString) / sizeof(TerminalTwoString[0]));
//					UART_WriteBlocking(DEMO_UART0, testBufferFour, sizeof(testBufferFour) / sizeof(testBufferFour[0]));
//					UART_WriteBlocking(DEMO_UART0, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//				}
//			}
//
//			if((ESCTERA == uart0Data) && (ESCTERA == uart3Data))
//			{
//				UART_WriteBlocking(DEMO_UART0, TerminalOneEndingString, sizeof(TerminalOneEndingString) / sizeof(TerminalOneEndingString[0]));
//				UART_WriteBlocking(DEMO_UART0, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//				UART_WriteBlocking(DEMO_UART3, TerminalOneEndingString, sizeof(TerminalOneEndingString) / sizeof(TerminalOneEndingString[0]));
//				UART_WriteBlocking(DEMO_UART3, NewLineString, sizeof(NewLineString) / sizeof(NewLineString[0]));
//				delay(600000);
//				vTaskDelete(chatHandle);
//			}
//		}
//		vTaskDelay(100);
//	}
//}

void eco_task(void *arg)
{
	vTaskSuspend(lcdHandle);
	LCDNokia_clear();
	LCDNokia_clear();
	uint8_t option = 0x31;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	struct netconn *newconn;
	newconn = (struct netconn*)(arg);
	for(;;)
	{
		netconn_write(newconn, "Press 0 to exit\n\r", 17, NETCONN_COPY);
		while ((err = netconn_recv(newconn, &buf)) == ERR_OK) {

			netbuf_data(buf, &data, &len);
			option = *(char*)data;
			if(option == 0x30)
			{
				netbuf_delete(buf);
				LCDNokia_clear();
				LCDNokia_clear();
				vTaskResume(lcdHandle);
				printingMenu((void *)newconn);
			}
			LCDNokia_sendChar(option);
			while (netbuf_next(buf) >= 0);
			netbuf_delete(buf);
		}
	}
}



void getTime_task(void *pvParameters)
{
	/*Start Timer*/

	I2C_RtcWrite(I2C0, RTC_DEVICE_ADD, 0x00, 0x80);

	for(;;)
	{
		if(FALSE == I2C_RtcRead(I2C0, RTC_DEVICE_ADD, 0x00, timeBuffer, 7))
		{
			UART_WriteBlocking(DEMO_UART0, errorString, sizeof(errorString) / sizeof(errorString[0]));
			delay(600000);
			UART_WriteBlocking(DEMO_UART0, deleteLine, sizeof(deleteLine) / sizeof(deleteLine [0]));
			UART_WriteBlocking(DEMO_UART0, adjut10, sizeof(adjut10) / sizeof(adjut10[0]));
		}

		I2C_RtcRead(I2C0, RTC_DEVICE_ADD, 0x00, timeBuffer, 7);
		timeBuffer[0] = timeBuffer[0] & SECONDS_REG_SIZE;
		timeBuffer[1] = timeBuffer[1] & MINUTES_REG_SIZE;
		timeBuffer[2] = timeBuffer[2] & HOURS_REG_SIZE;
		timeBuffer[4] = timeBuffer[4] & DAY_REG_SIZE;
		timeBuffer[5] = timeBuffer[5] & MONTH_REG_SIZE;
		timeBuffer[6] = timeBuffer[6] & YEAR_REG_SIZE;

		asciiDate[0] = ((timeBuffer[0] & BCD_L)) + ASCII_NUMBER_MASK;
		asciiDate[1] = ((timeBuffer[0] & BCD_H)>>4)+ASCII_NUMBER_MASK;
		asciiDate[2] = ((timeBuffer[1] & BCD_L)) + ASCII_NUMBER_MASK;
		asciiDate[3] = ((timeBuffer[1] & BCD_H)>>4)+ASCII_NUMBER_MASK;
		asciiDate[4] = ((timeBuffer[2] & BCD_L)) + ASCII_NUMBER_MASK;
		asciiDate[5] = ((timeBuffer[2] & BCD_H)>>4)+ASCII_NUMBER_MASK;
		asciiDate[6] = ((timeBuffer[4] & BCD_L))+ASCII_NUMBER_MASK;
		asciiDate[7] = ((timeBuffer[4] & BCD_H)>>4)+ASCII_NUMBER_MASK;
		asciiDate[8] = ((timeBuffer[5] & BCD_L))+ASCII_NUMBER_MASK;
		asciiDate[9] = ((timeBuffer[5] & BCD_H)>>4)+ASCII_NUMBER_MASK;
		asciiDate[10] = ((timeBuffer[6] & BCD_L))+ASCII_NUMBER_MASK;
		asciiDate[11] = ((timeBuffer[6] & BCD_H)>>4)+ASCII_NUMBER_MASK;
		vTaskDelay(1000);
		taskYIELD();
	}
}

void serialTimeLCD( void *pvParameters)
{
	for(;;)
	{

		LCDNokia_clear();
		LCDNokia_gotoXY(25,0);
		LCDNokia_sendString(HoraString);
		LCDNokia_gotoXY(16,1);
		LCDNokia_sendChar(asciiDate[5]);
		LCDNokia_sendChar(asciiDate[4]);
		LCDNokia_sendChar(':');
		LCDNokia_sendChar(asciiDate[3]);
		LCDNokia_sendChar(asciiDate[2]);
		LCDNokia_sendChar(':');
		LCDNokia_sendChar(asciiDate[1]);
		LCDNokia_sendChar(asciiDate[0]);

		LCDNokia_gotoXY(22,3);
		LCDNokia_sendString(fechaString);
		LCDNokia_gotoXY(8,4);
		LCDNokia_sendChar(asciiDate[7]);
		LCDNokia_sendChar(asciiDate[6]);
		LCDNokia_sendChar('-');
		LCDNokia_sendChar(asciiDate[9]);
		LCDNokia_sendChar(asciiDate[8]);
		LCDNokia_sendString(twentyCeroString);
		LCDNokia_sendChar(asciiDate[11]);
		LCDNokia_sendChar(asciiDate[10]);
		vTaskDelay(1000);

	}
}

uint16_t asciiToHex(uint8_t *string){
	while(*string){
		hexAddress = hexAddress << 4;
		if(*string >= 'A' && *string <= 'F'){
			hexAddress |= *string - ASCII_LETTER_MASK;
			*string++;
		}else{
			hexAddress |= *string - ASCII_NUMBER_MASK;
			*string++;
		}
	}
	return hexAddress;
}
