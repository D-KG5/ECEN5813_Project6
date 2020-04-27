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
#include "semphr.h"
#include <math.h>
/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "lookup.h"
#include "dma.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "global_defines.h"
#include "circ_buffer.h"
#include "led_control.h"
#include "fsl_dac.h"
#include "fsl_adc16.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
#define SW_TIMER_PERIOD_MS (1000 / portTICK_PERIOD_MS)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void task_DAC(void *pvParameters);
static void task_ADC(void *pvParameters);
static void handler_task(void *pvParameters);
static void task_calculate(void *pvParameters);
TaskHandle_t  Task_DAC = NULL;
TaskHandle_t  Task_ADC = NULL;
TaskHandle_t  Handler_task = NULL;
TaskHandle_t  calculation = NULL;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */

//timer_callback_dac
TimerHandle_t  timer_dac_handle= NULL;
static void timer_callback_dac(TimerHandle_t xTimer);
int start_dac = 0;

// timestamp counter vars for tenths, seconds, minutes, hours
uint8_t timestamp_counter_n = 0;
uint8_t timestamp_counter_s = 0;
uint8_t timestamp_counter_m = 0;
uint8_t timestamp_counter_h = 0;

static StaticQueue_t xADCStaticQueue;
uint8_t adcQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];
QueueHandle_t ADCBuffer;

int DAC_register_values[50];
int dac_counter = 0;

circ_buf_t * DSPBuffer;
uint32_t rawDSP_val[QUEUE_LENGTH] = {0};

extern SemaphoreHandle_t DMACntSemaphore;
extern SemaphoreHandle_t LEDBinSemaphore;

dma_transfer_config_t transferConfig;
extern dma_handle_t g_DMA_Handle;

TimerHandle_t  timer_log_handle = NULL;
static void timer_callback_log(TimerHandle_t xTimer);

//adc timer callback
TimerHandle_t  timer_adc_handle= NULL;
static void timer_callback_adc(TimerHandle_t xTimer);
int start_adc;

volatile bool g_Adc16ConversionDoneFlag = false;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t g_adc16ChannelConfigStruct;
uint32_t g_Adc16ConversionValue = 0;

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

static void timer_callback_dac(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
	start_dac = 1;
	taskEXIT_CRITICAL();
}

void ADC0_IRQHandler(void)
{
	taskENTER_CRITICAL();
    g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(ADC16_BASEADDR, ADC16_CHANNEL_GROUP);
    taskEXIT_CRITICAL();
}

static void timer_callback_adc(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
 	start_adc=1;
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

    EnableIRQ(ADC0_IRQn);
    dac_Init();
    adc_Init();
    dma_Init();

    DSPBuffer = init_buf(QUEUE_LENGTH * ITEM_SIZE);

	LEDBinSemaphore = xSemaphoreCreateBinary();
    if(LEDBinSemaphore == NULL){
    	PRINTF("FAIL\n");
    	LED_on(RED);
    }

    ADCBuffer = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, adcQueueStorageArea, &xADCStaticQueue);
    if(ADCBuffer == NULL){
    	PRINTF("FAIL\n");
    	LED_on(RED);
    }

    DMACntSemaphore = xSemaphoreCreateCounting(40, 0);
    if(DMACntSemaphore == NULL){
    	PRINTF("FAIL\n");
    	LED_on(RED);
    }

	timer_dac_handle = xTimerCreate("timer_callback_dac",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_dac);
	timer_adc_handle = xTimerCreate("timer_callback_adc",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_adc);
	timer_log_handle = xTimerCreate("timer_callback_log",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_log);

	if(timer_dac_handle== NULL && timer_adc_handle==NULL)
	{
		PRINTF("Fails");
		LED_on(RED);
	}

	else
	{
	    xTaskCreate(handler_task, "Handler", 1000, NULL, 0, &Task_DAC);
	    xTaskCreate(task_DAC, "DAC_task", 500, NULL, 0, &Task_ADC);
	    xTaskCreate(task_ADC, "ADCDMA_task", configMINIMAL_STACK_SIZE + 100, NULL, 0, &Handler_task);
	    xTaskCreate(task_calculate, "calculation_task", 500, NULL, 1,&calculation);

	    xTimerStart(timer_log_handle, 0);
		xTimerStart(timer_dac_handle, 0);
		xTimerStart(timer_adc_handle, 0);
		vTaskStartScheduler();
	}

    for (;;)
        ;
}


/*!
 * @brief Task responsible for DAC.
 */
static void task_DAC(void *pvParameters)
{
    for (;;)
    {
    	dac_voltagevalue();
    	if(start_dac==1)
    	{
    		LED_on(GREEN);
			taskENTER_CRITICAL();
			DAC_SetBufferValue(DAC_BASEADDR, 0U, DAC_register_values[dac_counter]);
//			PRINTF("DAC Value: %d\r\n", DAC_register_values[dac_counter]);
			dac_counter++;

			if(dac_counter == 49){
				dac_counter = 0;
			}
			taskEXIT_CRITICAL();
			LED_off(GREEN);
			start_dac=0;
    	}
    	taskYIELD();
    }

}


/*!
 * @brief Task responsible for ADC.
 */
static void task_ADC(void *pvParameters)
{
	BaseType_t ADCbufstatus;
	for(;;){
    	if(start_adc==1)
    	{
    		g_Adc16ConversionDoneFlag = false;
            ADC16_SetChannelConfig(ADC16_BASEADDR, ADC16_CHANNEL_GROUP, &g_adc16ChannelConfigStruct);

            while (!g_Adc16ConversionDoneFlag)
            {
            }
            start_adc=0;
    	}

		vTaskDelay(100);
		// send to ADCBuffer
		ADCbufstatus = xQueueSendToBack(ADCBuffer, &g_Adc16ConversionValue, 0);
		if(ADCbufstatus != pdPASS){
			// buffer full, initiate DMA transfer
			LED_flash(BLUE, 1);
			taskENTER_CRITICAL();
			DMA_PrepareTransfer(&transferConfig, adcQueueStorageArea, sizeof(adcQueueStorageArea[0]), DSPBuffer->buffer,
					sizeof(DSPBuffer->buffer[0]), sizeof(adcQueueStorageArea), kDMA_MemoryToMemory);
			DMA_SubmitTransfer(&g_DMA_Handle, &transferConfig, kDMA_EnableInterrupt);

			DMA_StartTransfer(&g_DMA_Handle);
			Log_string("Started DMA Transfer\r\n", MAIN, LOG_STATUS);
			DSPBuffer->size = (QUEUE_LENGTH * ITEM_SIZE);
			DSPBuffer->tail = (QUEUE_LENGTH * ITEM_SIZE) ;
			taskEXIT_CRITICAL();
		}
		taskYIELD();
	}
}

/*!
 * @brief Task responsible for handling DMA interrupts.
 */
static void handler_task(void *pvParameters){
	uint8_t DSP_val[QUEUE_LENGTH * ITEM_SIZE] = {0};

	int counter = 1;
	for(;;){
		xSemaphoreTake(DMACntSemaphore, portMAX_DELAY);
		taskENTER_CRITICAL();
		// empty queue
		xQueueReset(ADCBuffer);
		Log_string("Finished DMA Transfer\r\n", MAIN, LOG_STATUS);

		// get data from DSP buffer
		for(int i = 0; i < (QUEUE_LENGTH * ITEM_SIZE); i++){
			DSP_val[i] = remove_item(DSPBuffer);
		}
		DSPBuffer->head = 0;
		DSPBuffer->tail = 0;
		// fill int32 buffer
		for(int j = 0; j < QUEUE_LENGTH; j++){
			rawDSP_val[j] = (DSP_val[3 + (j * ITEM_SIZE)] << 24) + (DSP_val[2 + (j * ITEM_SIZE)] << 16) + (DSP_val[1 + (j * ITEM_SIZE)] << 8) + (DSP_val[0 + (j * ITEM_SIZE)] << 0);
//			PRINTF("Run: %d DSP %d: %u\r\n", counter, j, rawDSP_val[j]);
		}
		PRINTF("Run: %d\r\n", counter);
		// do processing and report 5 times, then end program
		xTaskNotify(calculation,0,eNoAction);//notifies the calculation task
		counter++;
		taskEXIT_CRITICAL();
		if(counter < 6){
			taskYIELD();
		}else{
			PRINTF("\r\nSTOPPED\r\n");
			taskENTER_CRITICAL();
			vTaskDelete(Task_DAC);
			vTaskDelete(Task_ADC);
			vTaskDelete(calculation);
			vTaskDelete(NULL);
			for(;;);
			taskEXIT_CRITICAL();
		}
	}
}

/*!
 * @brief Task responsible for math processing.
 */
static void task_calculate(void *pvParameters)
{
	float voltageval[QUEUE_LENGTH] = {0};
	for(;;)
	{
	   xTaskNotifyWait(0, 0, NULL,portMAX_DELAY );
	   taskENTER_CRITICAL();

	for(int k=0; k < (QUEUE_LENGTH + 1) ;k++)
	{
		voltageval[k] = (float)(rawDSP_val[k] * (VREF_BRD / SE_12BIT));
	}
	//inspired from https://www.programmingsimplified.com/c/source-code/c-program-find-maximum-element-in-array
	float maximum = voltageval[0];
	int location;

	for (int c = 0; c < QUEUE_LENGTH; c++)
	{
		if (voltageval[c] > maximum)
		{
			maximum  = voltageval[c];
			location = c;
		}
	}
	 PRINTF("Maximum element is present at location %d and its value is %0.3f\r\n", location, maximum);

	 float minimum = voltageval[0];


	for (int c = 0; c < QUEUE_LENGTH; c++)
	{
		if (voltageval[c] < minimum)
		{
			minimum  = voltageval[c];
			location = c;
		}
	}

	  PRINTF("Minimum element is present at location %d and its value is %0.3f\r\n", location, minimum);

	float sum=0.0;
	float average;
	for(int p=0; p<QUEUE_LENGTH; p++)
	{
		sum += voltageval[p];
	}
	average = sum / QUEUE_LENGTH;

	PRINTF("Average: %0.3f\r\n", average);

	//inspired by https://www.programiz.com/c-programming/examples/standard-deviation

	float SD = 0.0;
	for(int p = 0; p < QUEUE_LENGTH ;p++)
	{
		SD += pow((voltageval[p] - average), 2);
	}
	SD = sqrt(SD / QUEUE_LENGTH);

	PRINTF("Standard deviation: %0.3f\r\n\r\n", SD);
	taskEXIT_CRITICAL();
	taskYIELD();
	}
}


/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static – otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task’s
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task’s stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*———————————————————–*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static – otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task’s state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task’s stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
