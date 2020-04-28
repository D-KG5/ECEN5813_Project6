/*
 * lookup.c
 *
 *  Created on: Apr 16, 2020
 *      Author: sagar
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include <stdio.h>
#include <stdint.h>
#include "board.h"
#include "MKL25Z4.h"
#include "global_defines.h"
#include "lookup.h"
#include "led_control.h"
#include <math.h>

//code for voltage
extern int DAC_register_values[50];

// load values into lookup table
void dac_voltagevalue()
{
	static double step=0;
	float voltage=0;

	for (int i=0;i<50;i++)
	{
     voltage=(2+(sin(.4*PI*step) ));
    // PRINTF("hello %f\n",voltage);
     DAC_register_values[i]= (int) ((SE_12BIT*voltage/VREF_BRD)-1);
     step=step+0.1;
	}
}


void dac_Init(void)
{
	dac_config_t dacConfigStruct;
    DAC_GetDefaultConfig(&dacConfigStruct);
    DAC_Init(DAC_BASEADDR, &dacConfigStruct);
    DAC_Enable(DAC_BASEADDR, true); /* Enable output. */

}


void adc_Init(void)
{
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(ADC16_BASEADDR, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(ADC16_BASEADDR, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC16_BASEADDR))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
        LED_on(RED);
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
    g_adc16ChannelConfigStruct.channelNumber = ADC16_USER_CHANNEL;
    g_adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    g_adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */


}
