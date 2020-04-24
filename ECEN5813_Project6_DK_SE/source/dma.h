/*
 * dma.h
 *
 *  Created on: Apr 23, 2020
 *      Author: Dhruva
 */

#ifndef DMA_H_
#define DMA_H_

#include "fsl_dma.h"
#include "fsl_dmamux.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#define BUFF_LENGTH 4
#define DMA_CHANNEL 0
#define DMA_SOURCE 63

#define QUEUE_LENGTH 64
#define ITEM_SIZE sizeof(int)

void dma_Init(void);
void DMA_callback(dma_handle_t *handle, void *param);

#endif /* DMA_H_ */
