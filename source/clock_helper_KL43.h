/*
 * clock_helper.h
 *
 *  Created on: 4 may. 2022
 *      Author: Franco
 */

#ifndef CLOCK_HELPER_KL43_H_
#define CLOCK_HELPER_KL43_H_
#include "fsl_smc.h"

/**
 * Setea el clock del procesador a VLPR
 */
void APP_SetClockVlpr(void);

/**
 * Vuelve el clock del procesador a modo RUN
 */
void APP_SetClockRunFromVlpr(void);

/*
 * Muestra en qué modo está el procesador
 */
void APP_ShowPowerMode(smc_power_state_t currentPowerState);

#endif /* CLOCK_HELPER_KL43_H_ */
