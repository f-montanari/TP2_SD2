/*
 * clock_helper.c
 *
 *  Created on: 4 may. 2022
 *      Author: Franco
 */
#include "clock_config.h"
#include "fsl_smc.h"


void APP_SetClockVlpr(void)
{
    const sim_clock_config_t simConfig = {
        .pllFllSel = 0U,        /* PLLFLLSEL select FLL */
        .er32kSrc = 3U,         /* ERCLK32K selection, use LPO */
        .clkdiv1 = 0x00040000U, /* SIM_CLKDIV1 */
    };

    CLOCK_SetSimSafeDivs();
    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcFast, 0U);

    /* MCG works in PEE mode now, will switch to BLPI mode. */

    CLOCK_ExternalModeToFbeModeQuick();  /* Enter FBE. */
    CLOCK_SetFbiMode(kMCG_Dmx32Default, kMCG_DrsLow, NULL); /* Enter FBI. */
    CLOCK_SetLowPowerEnable(true);       /* Enter BLPI. */

    CLOCK_SetSimConfig(&simConfig);

    SMC_SetPowerModeVlpr(SMC);
}

void APP_SetClockRunFromVlpr(void)
{
    const sim_clock_config_t simConfig = {
        .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
        .er32kSrc = 3U,         /* ERCLK32K selection, use LPO */
        .clkdiv1 = 0x10010000U, /* SIM_CLKDIV1 */
    };

    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U,
		.prdiv = 0x1U,
		.vdiv = 0x0U,
    };

    SMC_SetPowerModeRun(SMC);

    CLOCK_SetSimSafeDivs();

    /* Currently in BLPI mode, will switch to PEE mode. */
    /* Enter FBI. */
    CLOCK_SetLowPowerEnable(false);
    /* Enter FBE. */
    CLOCK_SetFbeMode(3U, kMCG_Dmx32Default, kMCG_DrsLow, NULL);
    /* Enter PBE. */
    CLOCK_SetPbeMode(kMCG_PllClkSelPll0, &pll0Config);
    /* Enter PEE. 48 MHz */
    CLOCK_SetPeeMode();

    CLOCK_SetSimConfig(&simConfig);
}
