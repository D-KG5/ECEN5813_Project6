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
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define QUEUE_LENGTH 64
#define ITEM_SIZE sizeof(int)

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

static StaticQueue_t xADCStaticQueue;
uint8_t adcQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];
QueueHandle_t ADCBuffer;
static StaticQueue_t xDSPStaticQueue;
uint8_t dspQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];
QueueHandle_t DSPBuffer;

extern SemaphoreHandle_t DMACntSemaphore;

extern dma_transfer_config_t transferConfig;
extern dma_handle_t g_DMA_Handle;
extern volatile bool g_Transfer_Done;
uint32_t srcAddr[BUFF_LENGTH] = {0x01, 0x02, 0x03, 0x04};
uint32_t destAddr[BUFF_LENGTH] = {0x00, 0x00, 0x00, 0x00};

int DAC_register_values[50];

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

static void timer_callback_dac(TimerHandle_t xTimer)
{
   // taskENTER_CRITICAL();
	PRINTF("PRINT from CallBack\n\r");
	start_dac=1;
	//taskEXIT_CRITICAL();
}



int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    SystemCoreClockUpdate();
    dac_Init();
    dma_Init();

    ADCBuffer = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, adcQueueStorageArea, &xADCStaticQueue);
    if(ADCBuffer == NULL){
    	PRINTF("FAIL\n");
    }
    DSPBuffer = xQueueCreateStatic(QUEUE_LENGTH, ITEM_SIZE, dspQueueStorageArea, &xDSPStaticQueue);
    if(DSPBuffer == NULL){
    	PRINTF("FAIL\n");
    }

    DMACntSemaphore = xSemaphoreCreateCounting(10, 0);
    if(DMACntSemaphore == NULL){
    	PRINTF("FAIL\n");
    }

	timer_dac_handle = xTimerCreate("timer_callback_dac",pdMS_TO_TICKS(100),pdTRUE,NULL,timer_callback_dac);

	if(timer_dac_handle== NULL)
	{
		PRINTF("Fails");
	}
	else
	{
	    xTaskCreate(handler_task, "Handler", 1000, NULL, 3, NULL);
	    xTaskCreate(task_one, "Hello_task_one", 500, NULL, 1, NULL);
	    xTaskCreate(task_two, "Hello_task_two", configMINIMAL_STACK_SIZE + 10, NULL, 1, NULL);
	xTimerStart(timer_dac_handle, 0);
	vTaskStartScheduler();
	//init DMA with interrupt
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
    	 taskENTER_CRITICAL();
    	 PRINTF("Hello world.\r\n");
    	 taskEXIT_CRITICAL();
    	 start_dac=0;

    	}

     //   vTaskSuspend(NULL);
    }

}
// ADC task
static void task_two(void *pvParameters)
{
	BaseType_t ADCbufstatus;
//	int valtosend = 4567;
	while(1)
	{
		taskENTER_CRITICAL();
       PRINTF("Hello Task two\r\n");
       taskEXIT_CRITICAL();

       vTaskDelay(100);
       //vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement)
     //  taskYIELD();
       // send to ADCBuffer
//       ADCbufstatus = xQueueSendToBack(ADCBuffer, &valtosend, 0);
       DMA_SubmitTransfer(&g_DMA_Handle, &transferConfig, kDMA_EnableInterrupt);
       DMA_StartTransfer(&g_DMA_Handle);
       if(ADCbufstatus != pdPASS){
    	   // buffer full, initiate DMA transfer
//    	    DMA_StartTransfer(&g_DMA_Handle);
       }
	}
}

//
static void handler_task(void *pvParameters){
	int val[ITEM_SIZE + 1];
	int counter = 0;
	BaseType_t DSPbufstatus;
	for(;;){
		xSemaphoreTake(DMACntSemaphore, portMAX_DELAY);
		taskENTER_CRITICAL();
		PRINTF("In handler task\r\n");
		for (int i = 0; i < BUFF_LENGTH; i++)
		{
			PRINTF("%d\t", destAddr[i]);
		}
		taskEXIT_CRITICAL();
		// get data from DSP buffer
		DSPbufstatus = xQueueReceive(DSPBuffer, val, portMAX_DELAY);
		if(DSPbufstatus == pdPASS){
			PRINTF("DSP %d: %d\r\n", counter, val);
			counter++;
		}else{
			PRINTF("Got nothing in DSP\r\n");
		}
		// do processing and report 5 times, then end program
	}
}
