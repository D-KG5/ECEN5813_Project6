/*
 * dac.c
 *
 *  Created on: Apr 27, 2020
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
#include "dac.h"
#include <math.h>

//code for voltage
extern int DAC_register_values[50];

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



