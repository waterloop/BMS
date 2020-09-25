/*
 * ltc2949.c
 *
 * Used to interface with the LTC2949 battery monitoring chip found on the Master BMS.
 *
 * Created on: Sep 24, 2020
 * Author: Waterloop
 */

#include "ltc2949.h"
#include <stdbool.h>

/*
	\brief This function will initialize all 2949 variables and the SPI port.

	This function will initialize the Linduino to communicate with the LTC2949 with a 0.5MHz SPI clock.
*/
void LTC2949_Init(byte cellMonitorCount, bool ltc2949onTopOfDaisychain, bool debugEnable)
{
	LTC2949_CellMonitorCount = cellMonitorCount;

	LTC2949_DebugEnable = debugEnable;
	LTC2949_onTopOfDaisychain = ltc2949onTopOfDaisychain;

	// make sure the default value adjusted to e.g. BCREN is written with the next transaction
	LTC2949_forceWrRegsCtrl = true;
	LTC2949_autoForceWrRegsCtrl = true;
	LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] = LTC2949_BM_REGSCTRL_RDCVCONF;
	//LTC2949_init_device_state(); // don't do this here, as it also resets local stored device configuration (e.g. LTC2949_TBFAC,ADCConfig,GpioCurrConfig)

	spiBufferLen = 0;
	spiBufferBusy = true;
}

void LTC2949_Select_Read_Mode(byte cellMonitorCount, boolean ltc2949onTopOfDaisychain)
{
	LTC2949_forceWrRegsCtrl = true;
	LTC2949_autoForceWrRegsCtrl = true;
	LTC2949_CellMonitorCount = cellMonitorCount;
	LTC2949_onTopOfDaisychain = ltc2949onTopOfDaisychain;
}

/* Apply GPIO configuration to global variable LTC2949_gpioCtrl.
*  Call this function one ore more times to apply configuration for
*  one or more GPIOs. Afterwards call function
*  "LTC2949_GpioCurrConfigWrite" to write the actual
*  configuration to LTC2949
*/
void LTC2949_GPIO_Config(byte gpio, byte mode)
{
	// gpio  i  ADDR    REGNAME  	  BIT   BITNAME
	// 5     0  0xF1    FCURGPIOCTRL  1:0   GPO5CTRL
	// 1     1  0xF2    FGPIOCTRL	  1:0   GPO1CTRL
	// 2                              3:2   GPO2CTRL
	// 3                              5:4   GPO3CTRL
	// 4                              7:6   GPO4CTRL

	byte i;
	if (gpio == 0)
		return; // not allowed, ignore!
	if (gpio < 5)
	{
		i = 1;
		gpio--;
	}
	else if (gpio == 5)
	{
		i = 0;
		gpio = 0;
	}
	else
		return; // not allowed, ignore!

	gpio <<= 1U;

	LTC2949_gpioCtrl[i] &= ~(3U << gpio);
	LTC2949_gpioCtrl[i] |= mode << gpio;
}
