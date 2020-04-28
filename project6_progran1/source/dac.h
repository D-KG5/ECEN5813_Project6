/*
 * dac.h
 *
 *  Created on: Apr 27, 2020
 *      Author: sagar
 */

#ifndef DAC_H_
#define DAC_H_

#include "fsl_dac.h"
#include <stdio.h>
#include <stdint.h>
//dac
#define PI 3.141592653589793
#define VREF_BRD 3.300
#define SE_12BIT 4096.0
#define DEMO_DAC_BASEADDR DAC0

//adc
#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 0U /* PTE20, ADC0_SE0 */

//calculated values using matlab .m file
//t=0:0.1:5;
//y=(2+(1*sin(2*pi*(t/5))))
//Sine wave starts from 2V and amplitude is 1v

void dac_voltagevalue();
void dac_Init(void);




static const uint32_t lookup[50]=

   { 2000  ,  2125  ,  2248,    2368  ,  2481 ,   2587  ,  2684,    2770  ,  2844,    2904  ,  2951,    2982,
    2998   , 2998  ,  2982,    2951  ,  2904   , 2844  ,  2770  ,  2684  ,  2587  ,  2481  ,  2368  ,  2248,
    2125    ,2000 ,   1874 ,   1751 ,   1631   , 1518 ,   1412   , 1315 ,   1229   , 1155 ,   1095   , 1048,
    1017    ,1002,    1002  ,  1017,    1048   , 1095,    1155    ,1229,    1315    ,1412,    1518    ,1631,
    1751   , 1874 };


#endif /* DAC_H_ */
