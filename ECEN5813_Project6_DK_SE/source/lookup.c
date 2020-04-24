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
#include "lookup.h"
#include <math.h>

//code for voltage

extern int DAC_register_values[50];


////variables
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue = 0;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t g_adc16ChannelConfigStruct;




void dac_voltagevalue()
{
static double step=0;
float voltage=0;
//float a=2.564;
	for (int i=0;i<50;i++)
	{

     voltage=(2+(sin(.4*PI*step) ));
    // PRINTF("hello %f\n",voltage);
     //PRINTF("%f\n",a);
     DAC_register_values[i]= (int) ((SE_12BIT*voltage/VREF_BRD)-1);
    // PRINTF("%d\n",DAC_register_values[i]);

     step=step+0.1;
	}
}

void dac_Init(void)
{
	dac_config_t dacConfigStruct;
    DAC_GetDefaultConfig(&dacConfigStruct);
    DAC_Init(DEMO_DAC_BASEADDR, &dacConfigStruct);
    DAC_Enable(DEMO_DAC_BASEADDR, true); /* Enable output. */

}


void adc_Init(void)
{


	adc16_config_t adc16ConfigStruct;

    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#if defined(BOARD_ADC_USE_ALT_VREF)
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR))
    {
        PRINTF("\r\nADC16_DoAutoCalibration() Done.");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
    g_adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    g_adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    g_adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */


}
