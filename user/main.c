/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h" // F2806x Headerfile
#include "F2806x_EPwm_defines.h"
#include "F2806x_Examples.h" // F2806x Examples Headerfile
#include "SPLL_1ph_F.h"
#include "SPLL_1ph_SOGI_F.h"
#include "Solar_F.h"
#include "adc.h"
#include "epwm.h"
#include "key.h"
#include "math.h"
#include "timer.h"

#define Kp 22.7089
#define Ki 1063.14
#define GRID_VPP 25
#define ISR_FREQUENCY 10100

#define CURRENT_PEAK 1
_Bool flag = 0;
float ratio = 0;

#define THETA_GRAPH_INDEX 100
#define GRID_CURRENT_INDEX 150
#define RECTIFIER_CURR_INDEX 150
float theta_graph[THETA_GRAPH_INDEX];
float grid_current_graph[GRID_CURRENT_INDEX];

Uint16 compare1, compare2;

Uint16 theta_index;
Uint16 current_index;
float power_factor = 0.5;
float rectifier_theta;

float ref_current = 1.0;
float sineValue;
float error;
float output = 0;

float GRID_FREQ = 50;
extern float Vol1;
extern float Vol2;
float normalized_voltage;
float grid_voltage;
float grid_current;

float ref_sinwave;
Uint16 dutySin;
float dutyCycle;
float dutyCycle2;

extern float rectifier_voltage;

SPLL_1ph_SOGI_F spll1;

void LED_Init(void) {
  EALLOW;
  //    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;
  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
  GpioDataRegs.GPASET.bit.GPIO0 = 1;
  EDIS;
}

int main() {
  InitSysCtrl();
  DINT;
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;
  InitPieVectTable();

  EALLOW;
  PieVectTable.ADCINT1 = &adc_isr;
  PieVectTable.EPWM7_INT = &epwm7_timer_isr;
  // PieVectTable.EPWM8_INT = &epwm8_timer_isr;
  EDIS;
  InitAdc(); // For this example, init the ADC
  AdcOffsetSelfCal();
  // ADC
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
  IER |= M_INT1;                     // Enable CPU Interrupt 1
  EINT;
  ERTM;
  PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
  IER |= M_INT8;
  EINT;

  ADC_Init();
  LED_Init();
  InitPWM2();
  InitPWM3();
  InitPWM7();
  InitPWM8();
  KEY_Init();
  EALLOW;
  SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1; // ePWM7
  SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1; // ePWM8
  EDIS;

  IER |= M_INT3;
  PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx8 = 1;
  EINT; // Enable Global interrupt INTM
  ERTM; // Enable Global realtime interrupt DBGM

  SPLL_1ph_SOGI_F_init(GRID_FREQ, ((float)(1.0 / ISR_FREQUENCY)), &spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);

  spll1.lpf_coeff.B0_lf = (float32)((2 * Kp + Ki / ISR_FREQUENCY) / 2);
  spll1.lpf_coeff.B1_lf = (float32)(-(2 * Kp - Ki / ISR_FREQUENCY) / 2);

  TIM0_Init(90, 100); // 10K

  while (1) {
    if (flag == 0) {

      if (grid_voltage > 0) {
        // Set actions for rectifier
        EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;
        EPwm7Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm7Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        // Set actions for rectifier
        EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm8Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;

        // EPwm7Regs.CMPA.half.CMPA = MAX_CMPA;
        // EPwm8Regs.CMPA.half.CMPA = 0;
      }
      if (grid_voltage < 0) {
        // Set actions for rectifier
        EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm7Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm7Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        // Set actions for rectifier
        EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;
        EPwm8Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;

        // EPwm7Regs.CMPA.half.CMPA = 0;
        // EPwm8Regs.CMPA.half.CMPA = MAX_CMPA;
      }
    }
    // GpioDataRegs.GPBSET.bit.GPIO43 = 1;
  }
}

interrupt void TIM0_IRQn(void) {
  EALLOW;
  GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;

  // grid_voltage = (Vol2 - 1.493) * 34.013;
  // grid_vol_graph[gridvindex++] = grid_voltage;
  // if (gridvindex > GRID_V_INDEX)
  //   gridvindex = 0;

  normalized_voltage = grid_voltage / 10;
  spll1.u[0] = normalized_voltage;
  SPLL_1ph_SOGI_F_FUNC(&spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);

  ref_current =
      (power_factor * sin(spll1.theta[0]) +
       sqrt(1 - power_factor * power_factor) * cos(spll1.theta[0]) * 1) *
      CURRENT_PEAK * 1.4142136;

  grid_current = 0;
  // grid_voltage = 30;
  // output = (ref_current - grid_current) * (-8.0) / grid_voltage * MAX_CMPA;
  // // A sine wave, should between -1 and 1
  output = (ref_current - grid_current) * (-8.0) / grid_voltage *
           MAX_CMPA; // A sine wave, should between -1 and 1

  if (output < -MAX_CMPA)
    output = -MAX_CMPA;
  if (output > MAX_CMPA)
    output = MAX_CMPA;
  if (flag == 1) {

    // Set actions for rectifier
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
    // Set actions for rectifier
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;
    compare1 = (Uint16)output;
    compare2 = (Uint16)(-1 * output);

    EPwm7Regs.CMPA.half.CMPA = (Uint16)compare1;
    EPwm8Regs.CMPA.half.CMPA = (Uint16)compare2;

    // Check SOGI
    dutyCycle = (abs(output) + 1.0) / 2.0 * MAX_CMPA;
    EPwm2Regs.CMPA.half.CMPA = (Uint16)dutyCycle;
  }
  // Check Phase Shift
  dutyCycle2 = (sin(spll1.theta[0]) + 1.0) / 2.0 * MAX_CMPA;
  EPwm3Regs.CMPA.half.CMPA = (Uint16)dutyCycle2;

  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}
