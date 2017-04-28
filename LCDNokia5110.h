/*
 * LCDNokia5110.h
 *
 *  Created on: Jun 11, 2014
 *      Author: Luis
 */

#ifndef LCDNOKIA5110_H_
#define LCDNOKIA5110_H_

#include "SPI.h"

/**sets the lcd width to 84 */
#define SCREENW 84
/**sets the lcd hight to 48 */
#define SCREENH 48
/**sets the lcd x axis to 84*/
#define LCD_X 84
/**sets the lcd y axis to 48*/
#define LCD_Y 48
/**sets the Data that's on the lcd as 1*/
#define LCD_DATA 1
/**sets the cmd from the lcd to 0*/
#define LCD_CMD 0
/**sets the pin of the cmd from the lcd to 3*/
#define DATA_OR_CMD_PIN 3
/**sets reset pin to 0*/
#define RESET_PIN 0U

/*!
 *
 * \brief This function configures the LCD
 * */
void LCDNokia_init(void);

/*!
 *
 * \brief This function It writes a byte in the LCD memory. The place of writting is the last place that was indicated by LCDNokia_gotoXY. In the reset state
 * the initial place is x=0 y=0
 * */
void LCDNokia_writeByte(uint8_t DataOrCmd, uint8_t data);

/*!
 *
 * \brief This function it clears all the figures in the LCD
 *
 * \param[in] uint8_t
 * \param[in] uint8_t
 * */
void LCDNokia_clear(void);

/*!
 *
 * \brief This function It is used to indicate the place for writing a new character in the LCD. The values that x can take are 0 to 84 and y can take values
 * from 0 to 5
 *
 * \param[in] x, is the x axis for the lcd
 * \param[in] y, is the y axis for the lcd
 * */
void LCDNokia_gotoXY(uint8_t x, uint8_t y);

/*!
 *
 * \brief This function allows to write a figure represented by constant array
 *
 * */
void LCDNokia_bitmap(const uint8_t*);

/*!
 *
 * \brief This function writes a character in the LCD
 *
 * \param[in] uint8_t, it writes on a determinate bit from the lcd
 * */
void LCDNokia_sendChar(uint8_t);

/*!
 *
 * \brief It write a string into the LCD
 *
 * \param[in] uint8_t, it writes a string on a determinate fild from the lcd
 * */
void LCDNokia_sendString(uint8_t*);

/*!
 *
 * \brief This function it is used in the initialisation routine for the lcd screen
 *
 * */
void LCD_delay(void);



#endif /* LCDNOKIA5110_H_ */
