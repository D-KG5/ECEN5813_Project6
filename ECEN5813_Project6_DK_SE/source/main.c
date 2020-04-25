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
static void task_one(void *pvParameters);

static void task_two(void *pvParameters);
static void handler_task(void *pvParameters);
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */

//timer_callback_dac
TimerHandle_t  timer_dac_handle= NULL;
static void timer_callback_dac(TimerHandle_t xTimer);
int start_dac;

// timestamp counter vars for tenths, seconds, minutes, hours
uint8_t timestamp_counter_n = 0;
uint8_t timestamp_counter_s = 0;
uint8_t timestamp_counter_m = 0;
uint8_t timestamp_counter_h = 0;

static StaticQueue_t xADCStaticQueue;
uint8_t adcQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];
QueueHandle_t ADCBuffer;

circ_buf_t * DSPBuffer;

extern SemaphoreHandle_t DMACntSemaphore;

dma_transfer_config_t transferConfig;
extern dma_handle_t g_DMA_Handle;

uint32_t srcAddr[BUFF_LENGTH] = {0x01, 0x02, 0x03, 0x04};
uint8_t destAddr[QUEUE_LENGTH * ITEM_SIZE];

TimerHandle_t  timer_log_handle= NULL;
static void timer_callback_log(TimerHandle_t xTimer);


//adc timer callback
TimerHandle_t  timer_adc_handle= NULL;
static void timer_callback_adc(TimerHandle_t xTimer);
int start_adc;


volatile bool g_Adc16ConversionDoneFlag = false;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t g_adc16ChannelConfigStruct;
int g_Adc16ConversionValue=0;

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


int DAC_register_values[50];

static void timer_callback_dac(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
//	PRINTF("PRINT from CallBack\n\r");
	start_dac=1;
	taskEXIT_CRITICAL();
}

static void timer_callback_adc(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
//	PRINTF("PRINT from CallBack\n\r");
//	start_adc=1;
	g_Adc16ConversionDoneFlag = true;
  //  g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
 //   g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);
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
    Log_level(LOG_DEBUG);
    dac_Init();
    adc_Init();
    dma_Init();

    DSPBuffer = init_buf(QUEUE_LENGTH * ITEM_SIZE);

    ADCBuffer = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, adcQueueStorageArea, &xADCStaticQueue);
    if(ADCBuffer == NULL){
    	PRINTF("FAIL\n");
    }

    DMACntSemaphore = xSemaphoreCreateCounting(40, 0);
    if(DMACntSemaphore == NULL){
    	PRINTF("FAIL\n");
    }

	timer_dac_handle = xTimerCreate("timer_callback_dac",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_dac);
	timer_adc_handle = xTimerCreate("timer_callback_adc",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_adc);
	timer_log_handle = xTimerCreate("timer_callback_adc",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_log);

	if(timer_dac_handle== NULL && timer_adc_handle==NULL)
	{
		PRINTF("Fails");
	}

	else
	{
	    xTaskCreate(handler_task, "Handler", 1000, NULL, 3, NULL);
	    xTaskCreate(task_one, "Hello_task_one", 500, NULL, 1, NULL);

	    xTaskCreate(task_two, "Hello_task_two", configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);

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
static void task_one(void *pvParameters)
{
    for (;;)
    {
    	dac_voltagevalue();



    	if(start_dac==1)
    	{
    	 for(int i=0;i<50;i++)
    	 	 {
    		 	 DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, DAC_register_values[i]);
    	 	 }
//    	 taskENTER_CRITICAL();
//    	 PRINTF("Doing DAC stuff\r\n");
//    	 taskEXIT_CRITICAL();
    	 start_dac=0;

    	}


//    	}
     //   vTaskSuspend(NULL);
    }

}




// ADC task
static void task_two(void *pvParameters)
{
	BaseType_t ADCbufstatus;
//	timer_adc_handle = xTimerCreate("timer_callback_adc",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_adc);
	//xTimerStart(timer_adc_handle, 0);
	for(;;){
    	if(g_Adc16ConversionDoneFlag)
    	{
            g_Adc16ConversionDoneFlag = false;
            ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &g_adc16ChannelConfigStruct);

//            while (!g_Adc16ConversionDoneFlag)
//            {
//            }
        	while (0U == (kADC16_ChannelConversionDoneFlag &
        						  ADC16_GetChannelStatusFlags(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP)));

         g_Adc16ConversionValue= ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);
//         taskENTER_CRITICAL();
//       	 PRINTF("Doing ADC stuff\r\n");
//       	 taskEXIT_CRITICAL();
         start_adc=0;
    	}

//		Log_string("Hello Task two\r\n", MAIN, LOG_DEBUG);
		vTaskDelay(100);
		// send to ADCBuffer
		ADCbufstatus = xQueueSendToBack(ADCBuffer, &g_Adc16ConversionValue, 0);
		if(ADCbufstatus != pdPASS){
			// buffer full, initiate DMA transfer
			taskENTER_CRITICAL();
			DMA_PrepareTransfer(&transferConfig, adcQueueStorageArea, sizeof(adcQueueStorageArea[0]), DSPBuffer->buffer,
					sizeof(DSPBuffer->buffer[0]), sizeof(adcQueueStorageArea), kDMA_MemoryToMemory);
			DMA_SubmitTransfer(&g_DMA_Handle, &transferConfig, kDMA_EnableInterrupt);
			DMA_StartTransfer(&g_DMA_Handle);
			Log_string("Started DMA Transfer\r\n", MAIN, LOG_DEBUG);
			DSPBuffer->size = (QUEUE_LENGTH * ITEM_SIZE);
			DSPBuffer->tail = (QUEUE_LENGTH * ITEM_SIZE) ;
			taskEXIT_CRITICAL();
		}
	}
}

//
static void handler_task(void *pvParameters){
	uint8_t DSP_val[QUEUE_LENGTH * ITEM_SIZE] = {0};
	int rawDSP_val[QUEUE_LENGTH] = {0};

	int counter = 1;
	for(;;){
		xSemaphoreTake(DMACntSemaphore, portMAX_DELAY);
		taskENTER_CRITICAL();
		Log_string("Finished DMA Transfer\r\n", MAIN, LOG_DEBUG);
		xQueueReset(ADCBuffer);
		taskEXIT_CRITICAL();
		// get data from DSP buffer
		taskENTER_CRITICAL();
		for(int i = 0; i < (QUEUE_LENGTH * ITEM_SIZE); i++){
			DSP_val[i] = remove_item(DSPBuffer);
		}
		DSPBuffer->head = 0;
		DSPBuffer->tail = 0;
		// fill int32 buffer
		for(int j = 0; j < QUEUE_LENGTH; j++){
			rawDSP_val[j] = (DSP_val[3 + (j * ITEM_SIZE)] << 24) + (DSP_val[2 + (j * ITEM_SIZE)] << 16) + (DSP_val[1 + (j * ITEM_SIZE)] << 8) + (DSP_val[0 + (j * ITEM_SIZE)] << 0);
			PRINTF("Run: %d DSP %d: %u\r\n", counter, j, rawDSP_val[j]);
		}
		counter++;
		taskEXIT_CRITICAL();
		if(counter < 6){
			taskYIELD();
		}else{
			PRINTF("\r\nSTOPPED\r\n");
			taskENTER_CRITICAL();
			for(;;);
			taskEXIT_CRITICAL();
		}
		// do processing and report 5 times, then end program
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
