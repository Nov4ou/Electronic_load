/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h"   // F2806x Headerfile
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
#define GRID_VPP 7
#define ISR_FREQUENCY 10100
_Bool flag = 0;

#define THETA_GRAPH_INDEX 100
float theta_graph[THETA_GRAPH_INDEX];
Uint16 theta_index;
float power_factor = PI / 3.0;
float rectifier_theta;

float GRID_FREQ = 50;
extern float Vol1;
extern float Vol2;
float normalized_voltage;
float grid_voltage;

float ref_sinwave;
float dutyCycle;

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

    GpioDataRegs.GPBSET.bit.GPIO43 = 1;
    if (KEY_Read() != 0) {
      DELAY_US(20000);
      if (KEY_Read() == 2)
        flag = 1 - flag;
      while (KEY_Read() != 0)
        ;
    }
    if (flag == 0) {
      EALLOW;
      // SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 0; // ePWM7
      // SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 0; // ePWM8
      // SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1; // ePWM7
      // SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1; // ePWM8
      // EDIS;
      EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and count down
                                                     // Set actions test
      EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
      EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // Clear PWM1A on event A, up count
      EPwm8Regs.AQCTLA.bit.CAD = AQ_SET; // Clear PWM on down count
      // EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
      // EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count
      // EALLOW;
      EPwm7Regs.CMPA.half.CMPA = MAX_CMPA / 2;
      EPwm8Regs.CMPA.half.CMPA = MAX_CMPA / 2;
      EDIS;
    }
  }
}

interrupt void TIM0_IRQn(void) {
  EALLOW;
  GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;

  normalized_voltage = grid_voltage / GRID_VPP;
  spll1.u[0] = normalized_voltage;
  SPLL_1ph_SOGI_F_FUNC(&spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);

  GRID_FREQ = spll1.fo;
  rectifier_theta = spll1.theta[0];
  // rectifier_theta = spll1.theta[0];
  // rectifier_theta = spll1.theta[0];
  // if (rectifier_theta > (2 * PI))
  //   rectifier_theta = (2 * PI) - rectifier_theta;
  dutyCycle = (sin(rectifier_theta) + 1.0) / 2.0 * MAX_CMPA;

  ref_sinwave = sin(rectifier_theta);
  theta_graph[theta_index++] = rectifier_theta;
  if (theta_index >= THETA_GRAPH_INDEX)
    theta_index = 0;
  EPwm2Regs.CMPA.half.CMPA = (Uint16)dutyCycle;
  // EPwm7Regs.CMPA.half.CMPA = (Uint16)dutyCycle;
  // EPwm7Regs.CMPA.half.CMPA = 2250;

  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}
