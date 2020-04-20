/*
 * lookup.h
 *
 *  Created on: Apr 16, 2020
 *      Author: sagar
 */

#ifndef LOOKUP_H_
#define LOOKUP_H_



#define PI 3.141592653589793
#define VREF_BRD 3.300
#define SE_12BIT 4096.0
//calculated values using matlab .m file
//t=0:0.1:5;
//y=(2+(1*sin(2*pi*(t/5))))
//Sine wave starts from 2V and amplitude is 1v

void dac_voltagevalue();


static const double lookup[50]=

   { 2.0000  ,  2.1253  ,  2.2487,    2.3681  ,  2.4818 ,   2.5878  ,  2.6845,    2.7705  ,  2.8443,    2.9048  ,  2.9511,    2.9823,
    2.9980   , 2.9980  ,  2.9823,    2.9511  ,  2.9048   , 2.8443  ,  2.7705  ,  2.6845  ,  2.5878  ,  2.4818  ,  2.3681  ,  2.2487,
    2.1253    ,2.0000 ,   1.8747 ,   1.7513 ,   1.6319   , 1.5182 ,   1.4122   , 1.3155 ,   1.2295   , 1.1557 ,   1.0952   , 1.0489,
    1.0177    ,1.0020,    1.0020  ,  1.0177,    1.0489   , 1.0952,    1.1557    ,1.2295,    1.3155    ,1.4122,    1.5182    ,1.6319,
    1.7513   , 1.8747 };








#endif /* LOOKUP_H_ */


