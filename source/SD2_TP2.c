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

/*
 * Como el acelerómetro cambia solo de frecuencia de reloj,
 * sólo tenemos que cambiar el del procesador.
 */
void toVLPR(){
	// Seteo procesador en modo VLPR
	APP_SetClockVlpr();

	// Tambien desactivamos la opción de interrupción en
	// DataReady, para que no "despierte" al procesador.
	disableDataInterrupt();
}


/*
 * Como el acelerómetro cambia solo de frecuencia de reloj,
 * sólo tenemos que cambiar el del procesador.
 */
void toRUN(){
	// Seteo procesador en modo RUN
	APP_SetClockRunFromVlpr();
}


double getAcc(int16_t accX,int16_t accY,int16_t accZ){
	return (pow(accX,2) + pow(accY,2) + pow(accZ,2));
}

/*==================[macros and definitions]=================================*/
#define MAXIMO_MILISEG 1000
#define MAXIMA_CUENTA    1000

/*==================[internal data declaration]==============================*/
int timerBlink = 500;
int timer = 10000;

/*================ [implementacion MEF] =====================================*/
typedef enum {
	REPOSO = 0,
	CAIDA_LIBRE,
	DISPLAY,
}MEF_State;


void MEF_Main_Init(){
	toVLPR();
}

void MEF_Main_tick(){
	static MEF_State estadoActual = REPOSO;

	// datos internos
	double accAnterior = 0, accMayor = 0, accMayorRaiz = 0;

	switch(estadoActual){
	case REPOSO:
		if(mma8451_getFreefall()){
			toRUN();
			mma8451_clearFreefall();
			accAnterior = 0;
			accMayor = 0;
			timerBlink = 500;
			PRINTF("A Caida Libre\n");
			estadoActual = CAIDA_LIBRE;
			enableDataInterrupt();
		}
		break;
	case CAIDA_LIBRE:
		// Blink led rojo
		if(timerBlink == 0){
			timerBlink = 500;
			board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_TOGGLE);
		}

		// Obtengo la aceleración
		int16_t accX = mma8451_getAcX();
		int16_t accY = mma8451_getAcY();
		int16_t accZ = mma8451_getAcZ();

		// Para detectar el choque, supongamos una caída vertical perfecta (aceleración Z = -1g = -9.8 m/s^2)
		// El choque se detectará cuando la aceleración sea positiva (es decir, pasará de -9.8 a 0 o más)
		// Como estamos comparando aceleraciones cuadradas, la diferencia será como mínimo de 9.8^2 = 96.04
		// Ponemos el "threshold" para la detección de choque a un poco más de la mitad, para que sea un
		// poco más sensible según el golpe.
		double acc = getAcc(accX /100.0 * 9.8, accY /100.0 * 9.8, accZ /100.0 * 9.8);
		double diferencia = abs(acc - accAnterior);
		//PRINTF("%f,%f,diferencia: %f\n",acc,accAnterior,diferencia);

		if(acc > accMayor) accMayor = acc;

		if(diferencia >= 50 && accAnterior != 0){
			timer = 10000;
			accMayorRaiz = sqrt(accMayor);
			board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
			estadoActual = DISPLAY;
			PRINTF("A Display\n");
		}
		accAnterior = acc;

		break;
	case DISPLAY:
		SegLCD_DisplayDecimal(accMayorRaiz);
		if(timer == 0 || board_getSw(BOARD_SW_ID_1))
		{
			toVLPR();
			estadoActual = REPOSO;
			PRINTF("A Reposo\n");
		}
		break;
	}
}

/*
 * @brief   Application entry point.
 */
int main(void) {

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

    /* inicializa interrupción de systick cada 1 ms */
	SysTick_Config(SystemCoreClock / 1000U);

	// Cambiamos la prioridad del SysTick porque es más
	// importante atender las del acelerómetro.
	NVIC_SetPriority(SysTick_IRQn,1);

	MEF_Main_Init();

    while(1) {
    	MEF_Main_tick();
    }
    return 0 ;
}

void SysTick_Handler(void)
{
   if (timerBlink)
       timerBlink--;
   if (timer)
	   timer--;
}
