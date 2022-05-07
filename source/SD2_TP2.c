/*
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    SD2_TP2.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "fsl_smc.h"

/* TODO: insert other include files here. */
#include "SD2_board.h"
#include "SD2_I2C.h"
#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "mma8451.h"
#include "Seg_LCD.h"
#include "fsl_lpsci.h"

#include "clock_helper.h"

/* TODO: insert other definitions and declarations here. */
typedef enum {
	REPOSO = 0,
	CAIDA_LIBRE,
	DISPLAY,
}MEF_State;

void toVLPR(){
	// Seteo el acelerómetro en modo bajo consumo

	// Seteo procesador en modo VLPR
	APP_SetClockVlpr();

}

double getAcc(int16_t accX,int16_t accY,int16_t accZ){
	return (pow(accX,2) + pow(accY,2) + pow(accZ,2));
}

double accAnterior = 0;

/*==================[macros and definitions]=================================*/
#define MAXIMO_MILISEG 1000
#define MAXIMA_CUENTA    1000

/*==================[internal data declaration]==============================*/
static uint16_t miliseg = 0;
static uint16_t contador = 0;

/*
 * @brief   Application entry point.
 */
int main(void) {

	// Esto es solo para testear ahora.
	// Habría que mantener una bandera en el driver
	// que indique en qué modo está trabajando, porque
	// va a intentar leer datos en medio de la configuracion
	// (lo cual es malo)
	bool midiendo = false, actualizo = false;
	double accMayor = 0;

    BOARD_InitBootClocks();

	// Se inicializan funciones de la placa
	board_init();

	/* =========== I2C =================== */
	SD2_I2C_init();

	/* =========== MMA8451 ================ */

	mma8451_init_freefall();

	/* =========== LCD ================ */
	SegLCD_Init();
    SegLCD_DP2_Off();
    SegLCD_DisplayDecimal(0);

    /* inicializa interrupción de systick cada 1 ms */
	SysTick_Config(SystemCoreClock / 1000U);
	NVIC_SetPriority(SysTick_IRQn,1);

    while(1) {
    	if(miliseg == 0)
		{
			miliseg = MAXIMO_MILISEG;

			contador++;
			if(contador == MAXIMA_CUENTA){
				contador = 0;
			}
			SegLCD_DisplayDecimal(contador);
		}

    	// TEMPORAL!
    	// Acá habría que implementar la MEF
    	if(mma8451_getFreefall()){
    		board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
    		mma8451_clearFreefall();

    		midiendo = true;
    		//mma8451_init_continuous();
    	}
    	if(midiendo){
    		if(board_getSw(BOARD_SW_ID_3)){
    			midiendo = false;
				board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
				disableDataInterrupt();
    		}else{
    			int16_t accX = mma8451_getAcX();
    			int16_t accY = mma8451_getAcY();
    			int16_t accZ = mma8451_getAcZ();

    			double acc = getAcc(accX /100.0 * 9.8, accY /100.0 * 9.8, accZ /100.0 * 9.8);
    			double diferencia = abs(acc - accAnterior);
    			PRINTF("%f,%f,diferencia: %f\n",acc,accAnterior,diferencia);
    			if(diferencia >= 50 && accAnterior != 0){
    				PRINTF("Modo display!\n");
    			}
    			accAnterior = acc;

    		}

    	}
    }
    return 0 ;
}

void SysTick_Handler(void)
{
   if (miliseg)
       miliseg--;
}
