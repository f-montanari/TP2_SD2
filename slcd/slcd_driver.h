/*
 * slcs_driver.h
 *
 *  Created on: 9 may. 2022
 *      Author: dante
 */

#ifndef SLCD_DRIVER_H_
#define SLCD_DRIVER_H_

// inclusions
#include "fsl_common.h"
#include "fsl_slcd.h"
#include "slcd_engine.h"

// definitions
#define APP_SLCD_DUTY_CYCLE          kSLCD_1Div4DutyCycle
#define APP_SLCD_LOW_PIN_ENABLED     0x0d10c000U /* LCD_P27/26/24/20 -> b27/26/24/20 = 1. */
#define APP_SLCD_HIGH_PIN_ENABLED    0x18001d00U /* LCD_P44/43/42/40 -> b12/11/10/8 = 1. */
#define APP_SLCD_BACK_PANEL_LOW_PIN  0x0000c000U /* LCD_P15/P14 -> b15/b14 = 1. */
#define APP_SLCD_BACK_PANEL_HIGH_PIN 0x18000000U /* LCD_P60/P59 -> b28/27 = 1. */

// prototypes
void SLCD_Driver_Init(void);
void BOARD_SetSlcdBackPlanePhase(void);
void SLCD_Driver_TimeDelay(uint32_t ms);
void SLCD_Driver_ShowDigital(double);
void SLCD_Driver_Show_Icon(void);
void SLCD_Driver_Blink(void);
void SLCD_Driver_Clear(void);

// variables
typedef enum {
	SD_SHOW_ERROR = 0, // display some kind of error
	SD_SHOW_0_DECIMALS, // display value in order 1000
	SD_SHOW_1_DECIMALS, // display value in order 100.0
	SD_SHOW_2_DECIMALS, // display value in order 10.00
	SD_SHOW_3_DECIMALS, // display value in order 1.000
}showDigital_state;

#endif /* SLCD_DRIVER_H_ */
