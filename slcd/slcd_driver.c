/*
 * slcd_driver.c
 *
 *  Created on: 9 may. 2022
 *      Author: dante
 */


#include "slcd_driver.h"
#include "LCD-S401M16KR.h"

slcd_config_t config;
tSLCD_Engine slcd_engine;
const uint8_t slcd_lcd_gpio_seg_pin[] = {
    20, /* SLCD P05 --- LCD_P20. */
    24, /* SLCD P06 --- LCD_P24. */
    26, /* SLCD P07 --- LCD_P26. */
    27, /* SLCD P08 --- LCD_P27. */
    40, /* SLCD P09 --- LCD_P40. */
    42, /* SLCD P10 --- LCD_P42. */
    43, /* SLCD P11 --- LCD_P43. */
    44, /* SLCD P12 --- LCD_P44. */
};
slcd_clock_config_t slcdClkConfig = {kSLCD_AlternateClk1, kSLCD_AltClkDivFactor256, kSLCD_ClkPrescaler01
#if FSL_FEATURE_SLCD_HAS_FAST_FRAME_RATE
                                     ,
                                     false
#endif
};

void BOARD_SetSlcdBackPlanePhase(void){
    SLCD_SetBackPlanePhase(LCD, 59, kSLCD_PhaseAActivate); /* SLCD COM1 --- LCD_P59. */
    SLCD_SetBackPlanePhase(LCD, 60, kSLCD_PhaseBActivate); /* SLCD COM2 --- LCD_P60. */
    SLCD_SetBackPlanePhase(LCD, 14, kSLCD_PhaseCActivate); /* SLCD COM3 --- LCD_P14. */
    SLCD_SetBackPlanePhase(LCD, 15, kSLCD_PhaseDActivate); /* SLCD COM4 --- LCD_P15. */
}

static void SLCD_SetLCDPin(lcd_set_type_t type, uint32_t lcd_pin, uint8_t pin_val, int32_t on){
    assert(lcd_pin > 0);

    uint8_t gpio_pin = 0;
    uint8_t bit_val  = 0;
    uint8_t i        = 0;

    /* lcd _pin starts from 1. */
    gpio_pin = slcd_lcd_gpio_seg_pin[lcd_pin - 1];

    if (type == SLCD_Set_Num){
        SLCD_SetFrontPlaneSegments(LCD, gpio_pin, (on ? pin_val : 0));
    }
    else{
        for (i = 0; i < 8; ++i){
            bit_val = (uint8_t)(pin_val >> i) & 0x1U;
            if (bit_val){
                SLCD_SetFrontPlaneOnePhase(LCD, gpio_pin, (slcd_phase_index_t)i, on);
            }
        }
    }
}

void SLCD_Driver_Init(){
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	/* Enable the MCGIRCLK */
	MCG->C1 |= MCG_C1_IRCLKEN_MASK;
	/* SLCD get default configure. */
	/*
	 * config.displayMode = kSLCD_NormalMode;
	 * config.powerSupply = kSLCD_InternalVll3UseChargePump;
	 * config.voltageTrim = kSLCD_RegulatedVolatgeTrim08;
	 * config.lowPowerBehavior = kSLCD_EnabledInWaitStop;
	 * config.frameFreqIntEnable = false;
	 * config.faultConfig = NULL;
	 */
	SLCD_GetDefaultConfig(&config);

	/* Verify and Complete the configuration structure. */
	config.clkConfig          = &slcdClkConfig;
	config.loadAdjust         = kSLCD_HighLoadOrSlowestClkSrc;
	config.dutyCycle          = APP_SLCD_DUTY_CYCLE;
	config.slcdLowPinEnabled  = APP_SLCD_LOW_PIN_ENABLED;
	config.slcdHighPinEnabled = APP_SLCD_HIGH_PIN_ENABLED;
	config.backPlaneLowPin    = APP_SLCD_BACK_PANEL_LOW_PIN;
	config.backPlaneHighPin   = APP_SLCD_BACK_PANEL_HIGH_PIN;
	config.faultConfig = NULL;
	SysTick_Config(SystemCoreClock/1000u);
	/* SLCD Initialize. */
	SLCD_Init(LCD, &config);
	BOARD_SetSlcdBackPlanePhase();
	memset(&slcd_engine, 0, sizeof(tSLCD_Engine));
	SLCD_Engine_Init(&slcd_engine, SLCD_SetLCDPin);
}

void SLCD_Driver_Clear(){
	for (int position = 0; position < ICON_END; position++){
		SLCD_Engine_Show_Icon(&slcd_engine, position, 0);
	}
}

void SLCD_Driver_ShowDigital(double value){
	showDigital_state state;
	uint8_t digitNum[4];

	if(value >= 10000){
		state = SD_SHOW_ERROR;
	}else if(value >= 1000){
		state = SD_SHOW_0_DECIMALS;
	}else if(value >= 100){
		state = SD_SHOW_1_DECIMALS;
	}else if(value >= 10){
		state = SD_SHOW_2_DECIMALS;
	}else{
		state = SD_SHOW_3_DECIMALS;
	}

	switch(state){
		case SD_SHOW_ERROR:
			// 2nd digit "E"
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_2A, 1);
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_2G, 1);
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_2D, 1);
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_2E, 1);
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_2F, 1);
			// 3rd digit "r"
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_3E, 1);
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_3G, 1);
			// 4th digit "r"
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_4E, 1);
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_4G, 1);
			break;
		case SD_SHOW_0_DECIMALS:
			value += 0.5;
			digitNum[0] = (uint8_t) (value / 1000.0); // MSD
			digitNum[1] = (uint8_t) ((value-digitNum[0]*1000.0) / 100.0);
			digitNum[2] = (uint8_t) ((value-digitNum[0]*1000.0-digitNum[1]*100.0) / 10.0);
			digitNum[3] = (uint8_t) ((value-digitNum[0]*1000.0-digitNum[1]*100.0-digitNum[2]*10.0)); // LSD
			for (int i = 0; i < NUM_POSEND; i++){
				SLCD_Engine_Show_Num(&slcd_engine, digitNum[i], i, 1);
			}
			break;
		case SD_SHOW_1_DECIMALS:
			value += 0.05;
			digitNum[0] = (uint8_t) (value / 100.0); // MSD
			digitNum[1] = (uint8_t) ((value-digitNum[0]*100.0) / 10.0);
			digitNum[2] = (uint8_t) ((value-digitNum[0]*100.0-digitNum[1]*10.0)); // .
			digitNum[3] = (uint8_t) ((value-digitNum[0]*100.0-digitNum[1]*10.0-digitNum[2]) * 10.0); // LSD
			for (int i = 0; i < NUM_POSEND; i++){
				SLCD_Engine_Show_Num(&slcd_engine, digitNum[i], i, 1);
			}
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_DP1+2, 1);
			break;
		case SD_SHOW_2_DECIMALS:
			value += 0.005;
			digitNum[0] = (uint8_t) (value / 10.0); // MSD
			digitNum[1] = (uint8_t) ((value-digitNum[0]*10.0)); // .
			digitNum[2] = (uint8_t) ((value-digitNum[0]*10.0-digitNum[1]) * 10.0);
			digitNum[3] = (uint8_t) ((value-digitNum[0]*10.0-digitNum[1]-digitNum[2]/10.0) * 100.0); // LSD
			for (int i = 0; i < NUM_POSEND; i++){
				SLCD_Engine_Show_Num(&slcd_engine, digitNum[i], i, 1);
			}
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_DP1+1, 1);
			break;
		case SD_SHOW_3_DECIMALS:
			value += 0.0005;
			digitNum[0] = (uint8_t) value; // MSD .
			digitNum[1] = (uint8_t) ((value-digitNum[0]) * 10.0);
			digitNum[2] = (uint8_t) ((value-digitNum[0]-digitNum[1]/10.0) * 100.0);
			digitNum[3] = (uint8_t) ((value-digitNum[0]-digitNum[1]/10.0-digitNum[2]/100.0) * 1000.0); // LSD
			for (int i = 0; i < NUM_POSEND; i++){
				SLCD_Engine_Show_Num(&slcd_engine, digitNum[i], i, 1);
			}
			SLCD_Engine_Show_Icon(&slcd_engine, ICON_DP1, 1);
			break;
	}
}

void SLCD_Driver_TimeDelay(uint32_t ms){
    SDK_DelayAtLeastUs(1000 * ms, SystemCoreClock);
}
