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
#include "circ_buffer.h"

SemaphoreHandle_t DMACntSemaphore;
dma_handle_t g_DMA_Handle;

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
}
