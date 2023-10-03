/*
 * display.h
 *
 *  Created on: Aug 5, 2023
 *      Author: user
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include <stdint.h>
#include <stdbool.h>


#include "../src/lcd/stm32_adafruit_lcd.h"
#include "../src/lcd/Fonts/fonts.h"

void DisplayInit(void);
void DisplayUpdate(void);
void RegisterButtonPress(uint32_t button);








#endif /* INC_DISPLAY_H_ */
