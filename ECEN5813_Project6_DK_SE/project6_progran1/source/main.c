/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * o Neither the name of the copyright holder nor the names of its
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

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "fsl_dac.h"
/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "dac.h"
#include "pin_mux.h"
#include "global_defines.h"
#include "semphr.h"
#include "led_control.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void dac_task(void *pvParameters);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */

// timestamp counter vars for tenths, seconds, minutes, hours
uint8_t timestamp_counter_n = 0;
uint8_t timestamp_counter_s = 0;
uint8_t timestamp_counter_m = 0;
uint8_t timestamp_counter_h = 0;

extern SemaphoreHandle_t LEDBinSemaphore;



TimerHandle_t  timer_log_handle = NULL;
static void timer_callback_log(TimerHandle_t xTimer);



static void timer_callback_log(TimerHandle_t xTimer){
	taskENTER_CRITICAL();
	timestamp_counter_n++;
	if(timestamp_counter_n == 10){
		timestamp_counter_n = 0;
		timestamp_counter_s++;
	}
	if(timestamp_counter_s == 60){
		timestamp_counter_s = 0;
		timestamp_counter_m++;
	}
	if(timestamp_counter_m == 60){
		timestamp_counter_m = 0;
		timestamp_counter_h++;
	}
	taskEXIT_CRITICAL();
}

//timer_callback_dac
TimerHandle_t  timer_dac_handle= NULL;
static void timer_callback_dac(TimerHandle_t xTimer);
int start_dac;
int DAC_register_values[50];
int dac_counter = 0;
static void timer_callback_dac(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
    LED_flash(BLUE,1);
	start_dac=1; //sets the flag to one
	taskEXIT_CRITICAL();
}





int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    SystemCoreClockUpdate();
    Log_enable();
    Log_level(LOG_STATUS);
    LED_init();
    dac_Init();
	LEDBinSemaphore = xSemaphoreCreateBinary();
    if(LEDBinSemaphore == NULL){
    	PRINTF("FAIL\n");
    	LED_on(RED);
    }



    timer_dac_handle = xTimerCreate("timer_callback_dac",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_dac);
    timer_log_handle = xTimerCreate("timer_callback_log",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_log);
	if(timer_dac_handle== NULL&& timer_log_handle==NULL)
	{
		PRINTF("Fails");
		LED_on(RED);
	}
	xTimerStart(timer_dac_handle, 0);
	xTimerStart(timer_log_handle, 0);
	Log_string("Started DAC Transfer\r\n", MAIN, LOG_STATUS);
    xTaskCreate(dac_task, "Hello_task", 500, NULL, 0, NULL);

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void dac_task(void *pvParameters)
{
    for (;;)
    {
    	//int voltage;
    	dac_voltagevalue();

    	if(start_dac==1)
    	{
    	//	LED_on(BLUE);
			taskENTER_CRITICAL();
			DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, DAC_register_values[dac_counter]);
            //voltage= 1000 * (DAC_register_values[dac_counter]+1)*(VREF_BRD/SE_12BIT);
//			if(dac_counter<51)
//			{
//            PRINTF("%d,\r\n",voltage);
//			}
			PRINTF("DAC Value[%d]: %d\r\n",dac_counter, DAC_register_values[dac_counter]);//prints the dac registe values
		//	Log_integer(DAC_register_values[dac_counter],MAIN,1);
			dac_counter++;

			if(dac_counter == 49){
				dac_counter = 0;
			}
			taskEXIT_CRITICAL();
		//	LED_off(BLUE);
			start_dac=0;//sets the flag to zero
    	}


        //PRINTF("Hello world.\r\n");
       // vTaskSuspend(NULL);
    }
}
