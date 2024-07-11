/*
 * adc.h
 *
 *  Created on: May 5, 2024
 *      Author: nov4ou
 */

#ifndef APP_ADC_ADC_H_
#define APP_ADC_ADC_H_

#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile


void ADC_Init();
__interrupt void adc_isr(void);

#endif /* APP_ADC_ADC_H_ */
