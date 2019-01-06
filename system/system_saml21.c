#include "saml21.h"

/**
 * Initial system clock frequency. The System RC Oscillator (RCSYS) provides
 *  the source for the main clock at chip startup.
 */
#define __SYSTEM_CLOCK    (12000000)

uint32_t SystemCoreClock = __SYSTEM_CLOCK;/*!< System Clock Frequency (Core Clock)*/

/**
 * Initialize the system
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
void SystemInit(void)
{
    uint32_t temp;

    //Read calibration values
    temp = *((uint32_t*) NVMCTRL_OTP5);
    ADC->CALIB.reg = ADC_CALIB_BIASREFBUF(temp) | ADC_CALIB_BIASCOMP(temp >> 3);
    OSC32KCTRL->OSC32K.reg = OSC32KCTRL_OSC32K_RESETVALUE | OSC32KCTRL_OSC32K_CALIB(temp >> 6);
    USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN(temp >> 13) | USB_PADCAL_TRANSP(temp >> 18) | USB_PADCAL_TRIM(temp >> 23);

    OSCCTRL->DFLLCTRL.reg = 0;
	while (OSCCTRL->STATUS.bit.DFLLRDY == 0);

    OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE(temp >> 26);
    while (OSCCTRL->STATUS.bit.DFLLRDY == 0);

	//Start 32kHz crystal
	OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_STARTUP(0x03) | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_RUNSTDBY | OSC32KCTRL_XOSC32K_ENABLE;
	while (OSC32KCTRL->STATUS.bit.XOSC32KRDY == 0);

	//Set up clock generator 2: XOSC32K -> 1024 Hz
	GCLK->GENCTRL[2].reg = GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_DIV(32) | GCLK_GENCTRL_GENEN;
    while(GCLK->SYNCBUSY.bit.GENCTRL2);

	//Setup DFLL48M
	GCLK->PCHCTRL[0].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN;

	OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP(5) | OSCCTRL_DFLLMUL_FSTEP(5) | OSCCTRL_DFLLMUL_MUL(46875);
	while (OSCCTRL->STATUS.bit.DFLLRDY == 0);

	OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_WAITLOCK | OSCCTRL_DFLLCTRL_QLDIS | OSCCTRL_DFLLCTRL_MODE | OSCCTRL_DFLLCTRL_ENABLE;
	while (OSCCTRL->STATUS.bit.DFLLRDY == 0);

	//Wait for lock of DFLL48M
	while((OSCCTRL->STATUS.bit.DFLLLCKC == 0) && (OSCCTRL->STATUS.bit.DFLLLCKF == 0));

    //Connect generic clock 1 to DFLL48
    GCLK->GENCTRL[1].reg = GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_GENEN;
    while (GCLK->SYNCBUSY.bit.GENCTRL1);

    //Connect generic clock 0 to generic clock 1: 48 Mhz -> 12 Mhz
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_DIV(4) | GCLK_GENCTRL_SRC_GCLKGEN1 | GCLK_GENCTRL_GENEN;
    while (GCLK->SYNCBUSY.bit.GENCTRL0);

    //Disable internal oscillator
    OSCCTRL->OSC16MCTRL.bit.ENABLE = 0;

    SystemCoreClock = __SYSTEM_CLOCK;

	return;
}

/**
 * Update SystemCoreClock variable
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void)
{
	// Not implemented
	SystemCoreClock = __SYSTEM_CLOCK;
	return;
}
