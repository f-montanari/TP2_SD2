#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "math.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_smc.h"
#include "key.h"
#include "SD2_I2C.h"
#include "mma8451.h" // Agregué instrucción de pre-procesador para que funcione en ambas
#include "Seg_LCD.h"
#include "fsl_lpsci.h"
#include "clock_helper.h"
#include "MKL46Z4.h"

/* TODO: insert other definitions and declarations here. */

/*
 * No solo bajamos la frecuencia del reloj, también desactivamos
 * las interrupciones DRDY que haga el acelerómetro (y de paso bajamos
 * la frecuencia de adquisición de datos) y limpiamos el LCD.
 */
void toVLPR(){
	// Seteo procesador en modo VLPR
	APP_SetClockVlpr();

	// Limpiamos LCD
	SegLCD_Clear();
	SegLCD_DP2_Off();

	// Tambien desactivamos la opción de interrupción en
	// DataReady, para que no "despierte" al procesador.
	mma8451_disableDataInterrupt();
}


/*
 * Como el acelerómetro cambia solo de frecuencia de reloj,
 * sólo tenemos que cambiar el del procesador.
 */
void toRUN(){
	// Seteo procesador en modo RUN
	APP_SetClockRunFromVlpr();
}

double getAcc2(int16_t accX,int16_t accY,int16_t accZ){
	return (pow(accX,2) + pow(accY,2) + pow(accZ,2));
}

/*==================[macros and definitions]=================================*/
#define MAXIMO_MILISEG 1000
#define MAXIMA_CUENTA    1000
#define BLINK_TIME 50

/*==================[internal data declaration]==============================*/
int timerBlink = BLINK_TIME;
int timer = 10000;
int8_t nroSeparado[3];

/*================ [implementacion MEF] =====================================*/
typedef enum {
	REPOSO = 0,
	CAIDA_LIBRE,
	DISPLAY,
}MEF_State;

/*
 * Debido a que la función PRINTF integrada no acepta el modificador "%.3f" para mostrar decimales
 * se tuvo que hacer esta función para poder mostrar en consola el valor de la aceleración.
 */
void separarDigitos(double nro){
	nro += 0.005; // para redondear a dos decimales
	nroSeparado[0] = (int8_t) nro % 10; // valor de la unidad
	nroSeparado[1] = (int8_t) ((nro - nroSeparado[0])*10) % 10; // primer decimal
	nroSeparado[2] = (int8_t) ((nro - nroSeparado[0] - nroSeparado[1]/10) * 100) % 10; // segundo decimal
}

void MEF_Main_Init(){
	toVLPR();
	PRINTF("\tA Reposo. fclk = %d MHz\n", CLOCK_GetFreq(kCLOCK_CoreSysClk)/(uint32_t)1E6);
}

void MEF_Main_tick(){
	static MEF_State estadoActual = REPOSO;

	// datos internos
	static double accAnterior, accMayor, accMayorRaiz;

	switch(estadoActual){
	case REPOSO:
		if(mma8451_getFreefall()){
			toRUN();
			mma8451_clearFreefall();
			accAnterior = 0;
			accMayor = 0;
			timerBlink = BLINK_TIME;
			PRINTF("\tA Caida Libre. fclk = %d MHz\n", CLOCK_GetFreq(kCLOCK_CoreSysClk)/(uint32_t)1E6);
			estadoActual = CAIDA_LIBRE;
			mma8451_enableDataInterrupt();
			accAnterior = 0;
		}
		break;
	case CAIDA_LIBRE:
		// Blink led rojo
		if(timerBlink == 0){
			timerBlink = BLINK_TIME;
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
		
		double acc = getAcc2(accX /100.0 * 9.8, accY /100.0 * 9.8, accZ /100.0 * 9.8);
		double diferencia = abs(acc - accAnterior);
		//PRINTF("%f,%f,diferencia: %f\n",acc,accAnterior,diferencia);

		if(acc > accMayor){
			accMayor = acc;
		}

		if(diferencia >= 50 && accAnterior != 0){
			timer = 10000;
			accMayorRaiz = sqrt(accMayor);
			board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
			estadoActual = DISPLAY;
			separarDigitos((accMayorRaiz/9.8));
			PRINTF("\tAceleración máxima: %d.%d%d\n", nroSeparado[0], nroSeparado[1], nroSeparado[2]);
			PRINTF("\tA Display\n");
		}
		accAnterior = acc;

		break;
	case DISPLAY:
		SegLCD_DisplayDecimal(accMayorRaiz*10 + 0.05); // Redondeo a 2 decimales.
		SegLCD_DP2_On();
		if(timer == 0 || board_getSw(BOARD_SW_ID_1)){
			estadoActual = REPOSO;
			toVLPR();
			PRINTF("\tA Reposo. fclk = %d MHz\n", CLOCK_GetFreq(kCLOCK_CoreSysClk)/(uint32_t)1E6);
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

void SysTick_Handler(void){
   if(timerBlink)
	   timerBlink--;

   if(timer)
	   timer--;
}

