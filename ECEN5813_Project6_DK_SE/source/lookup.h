/*
 * lookup.h
 *
 *  Created on: Apr 16, 2020
 *      Author: sagar
 */

#ifndef LOOKUP_H_
#define LOOKUP_H_

#include "fsl_dac.h"
#include "fsl_adc16.h"


//dac
#define PI 3.141592653589793
#define VREF_BRD 3.300
#define SE_12BIT 4096.0
#define DAC_BASEADDR DAC0

//adc
#define ADC16_BASEADDR ADC0
#define ADC16_CHANNEL_GROUP 0U
#define ADC16_USER_CHANNEL 0U /* PTE20, ADC0_SE0 */

extern adc16_config_t adc16ConfigStruct;
extern adc16_channel_config_t g_adc16ChannelConfigStruct;

//calculated values using matlab .m file
//t=0:0.1:5;
//y=(2+(1*sin(2*pi*(t/5))))
//Sine wave starts from 2V and amplitude is 1v

void dac_voltagevalue();
void dac_Init(void);
void adc_Init(void);


static const double lookup[50]=

   { 2.0000  ,  2.1253  ,  2.2487,    2.3681  ,  2.4818 ,   2.5878  ,  2.6845,    2.7705  ,  2.8443,    2.9048  ,  2.9511,    2.9823,
    2.9980   , 2.9980  ,  2.9823,    2.9511  ,  2.9048   , 2.8443  ,  2.7705  ,  2.6845  ,  2.5878  ,  2.4818  ,  2.3681  ,  2.2487,
    2.1253    ,2.0000 ,   1.8747 ,   1.7513 ,   1.6319   , 1.5182 ,   1.4122   , 1.3155 ,   1.2295   , 1.1557 ,   1.0952   , 1.0489,
    1.0177    ,1.0020,    1.0020  ,  1.0177,    1.0489   , 1.0952,    1.1557    ,1.2295,    1.3155    ,1.4122,    1.5182    ,1.6319,
    1.7513   , 1.8747 };


//static const unit32_t lookup[50]=
//
//   { 2000  ,  2125  ,  2248,    2368  ,  2481 ,   2587  ,  2684,    2770  ,  2844,    2904  ,  2951,    2982,
//    2998   , 2998  ,  2982,    2951  ,  2904   , 2844  ,  2770  ,  2684  ,  2587  ,  2481  ,  2368  ,  2248,
//    2125    ,2000 ,   1874 ,   1751 ,   1631   , 1518 ,   1412   , 1315 ,   1229   , 1155 ,   1095   , 1048,
//    1017    ,1002,    1002  ,  1017,    1048   , 1095,    1155    ,1229,    1315    ,1412,    1518    ,1631,
//    1751   , 1874 };





#endif /* LOOKUP_H_ */


