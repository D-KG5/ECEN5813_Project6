/*
 * dma.c
 *
 *  Created on: Apr 23, 2020
 *      Author: Dhruva
 */

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

#include <stdio.h>
#include <stdint.h>
#include "board.h"
#include "MKL25Z4.h"
#include <math.h>
#include "dma.h"

SemaphoreHandle_t DMACntSemaphore;
dma_transfer_config_t transferConfig;
dma_handle_t g_DMA_Handle;

extern QueueHandle_t ADCBuffer;
extern QueueHandle_t DSPBuffer;

extern uint8_t adcQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];
extern uint8_t dspQueueStorageArea[QUEUE_LENGTH * ITEM_SIZE];
extern uint32_t srcAddr[BUFF_LENGTH];
extern uint32_t destAddr[BUFF_LENGTH];

void DMA_callback(dma_handle_t *handle, void *param){
	BaseType_t xHiPriorityTaskWoken;

	xHiPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(DMACntSemaphore, &xHiPriorityTaskWoken);

	portYIELD_FROM_ISR(xHiPriorityTaskWoken);
}

void dma_Init(void){
	DMAMUX_Init(DMAMUX0);
    DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL, DMA_SOURCE);
    DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL);

    DMA_Init(DMA0);
    DMA_CreateHandle(&g_DMA_Handle, DMA0, DMA_CHANNEL);
    DMA_SetCallback(&g_DMA_Handle, DMA_callback, NULL);
	DMA_PrepareTransfer(&transferConfig, adcQueueStorageArea, sizeof(adcQueueStorageArea[0]), dspQueueStorageArea,
			sizeof(dspQueueStorageArea[0]), sizeof(adcQueueStorageArea), kDMA_MemoryToMemory);
}
