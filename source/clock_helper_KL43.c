/*
 * clock_helper.c
 *
 *  Created on: 4 may. 2022
 *      Author: Franco
 */
#include "clock_config.h"
#include "fsl_smc.h"


void APP_SetClockVlpr(void){
	const sim_clock_config_t simConfig = {
		.clkdiv1 = 0x00010000U, // SIM_CLKDIV1
#if (defined(FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION) && FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION)
        .er32kSrc = 0U, /* SIM_SOPT1[OSC32KSEL]. */
#endif
	};
	const mcglite_config_t mcgliteConfig = {
		.outSrc = kMCGLITE_ClkSrcLirc,
		.irclkEnableMode = kMCGLITE_IrclkEnable,
		.ircs = kMCGLITE_Lirc2M,
		.fcrdiv = kMCGLITE_LircDivBy1,
		.lircDiv2 = kMCGLITE_LircDivBy1,
		.hircEnableInNotHircMode = false,
	};
	CLOCK_SetSimSafeDivs();
	CLOCK_SetMcgliteConfig(&mcgliteConfig);
	CLOCK_SetSimConfig(&simConfig);
	SMC_SetPowerModeVlpr(SMC);
	//while (kSMC_PowerStateVlpr != SMC_GetPowerModeState(SMC)){}
}

void APP_SetClockRunFromVlpr(void){
	const sim_clock_config_t simConfig = {
#if (defined(FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION) && FSL_FEATURE_SIM_OPT_HAS_OSC32K_SELECTION)
        .er32kSrc = 0U, /* SIM_SOPT1[OSC32KSEL]. */
#endif
		.clkdiv1 = 0x00010000U, // SIM_CLKDIV1
	};
	const mcglite_config_t mcgliteConfig = {
		.outSrc = kMCGLITE_ClkSrcHirc,
		.irclkEnableMode = 0, // no se que hace
		.ircs = kMCGLITE_Lirc8M, // no se que poner en este caso
		.fcrdiv = kMCGLITE_LircDivBy1,
		.lircDiv2 = kMCGLITE_LircDivBy1,
		.hircEnableInNotHircMode = true,
	};
	SMC_SetPowerModeRun(SMC);
	//while (kSMC_PowerStateRun != SMC_GetPowerModeState(SMC)){}
	CLOCK_SetSimSafeDivs();
	CLOCK_SetMcgliteConfig(&mcgliteConfig);
	CLOCK_SetSimConfig(&simConfig);
}
