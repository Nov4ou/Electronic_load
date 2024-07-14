/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h" // F2806x Headerfile
#include "F2806x_EPwm_defines.h"
#include "F2806x_Examples.h" // F2806x Examples Headerfile
#include "F2806x_Gpio.h"
#include "SPLL_1ph_F.h"
#include "SPLL_1ph_SOGI_F.h"
#include "Solar_F.h"
#include "adc.h"
#include "epwm.h"
#include "key.h"
#include "math.h"
#include "timer.h"

//#pragma CODE_SECTION(TIM0_IRQn, "ramfuncs");

//extern Uint16 RamfuncsLoadStart;
//extern Uint16 RamfuncsLoadEnd;
//extern Uint16 RamfuncsRunStart;
//extern Uint16 RamfuncsLoadSize;

// #define Kp 22.7089
#define Kp 25.7089
// #define Ki 1063.14
#define Ki 100.0
#define GRID_VPP 25
#define ISR_FREQUENCY 20000

Uint8 scope_mode;
float CURRENT_PEAK = 0.7;
_Bool flag = 0;
float ratio = 0;

#define GERERNAL_GRAPH_INDEX 150
float general_graph[GERERNAL_GRAPH_INDEX];
Uint16 general_index;

#define THETA_GRAPH_INDEX 100
#define RECTIFIER_CURR_INDEX 150
#define OUTPUT_GRAPH_INDEX 500
float theta_graph[THETA_GRAPH_INDEX];
float output_graph[OUTPUT_GRAPH_INDEX];
Uint16 theta_index;
Uint16 output_index;

Uint16 compare1, compare2;
float Kp_set = -8.0;
float V_dc_feedback = 0;
float V_in_feedback = 0;

float power_factor = 1;
float rectifier_theta;

float ref_current;
float current_soft_start = 0.0;
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
  // memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
  // InitFlash();
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

  TIM0_Init(90, 50.5); // 10K

  while (1) {
    if (KEY_Read() != 0) {
      if (KEY_Read() == 1) {
        flag = 1 - flag;
        while (KEY_Read() == 1)
          ;
      }
    }

    if (flag == 0) {
      current_soft_start = 0;
      EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      if (sin(spll1.theta[0] + PI / 6.0) > 0) {
        // Set actions for rectifier
        EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm7Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm7Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        // Set actions for rectifier
        EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm8Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      }
      if (sin(spll1.theta[0] + PI / 6.0) < 0) {
        // Set actions for rectifier
        EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm7Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm7Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        // Set actions for rectifier
        EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm8Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      }
    }

    if (flag == 2) {
      EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
      EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
      if (sin(spll1.theta[0] + PI / 6.0) >= 0) {
        EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm7Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm7Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm8Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      }
      if (sin(spll1.theta[0] + PI / 6.0) < 0) {
        EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;
        EPwm7Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm7Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;
        EPwm8Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm8Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      }
    }
  }
}

interrupt void TIM0_IRQn(void) {
  EALLOW;

  GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
  normalized_voltage = grid_voltage / 20;
  spll1.u[0] = normalized_voltage;
  SPLL_1ph_SOGI_F_FUNC(&spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);

  // ref_current =
  //     (power_factor * sin(spll1.theta[0] + PI / 6.0) +
  //      sqrt(1 - power_factor * power_factor) * cos(spll1.theta[0] + PI / 6.0)
  //      * 1) *
  //     CURRENT_PEAK * 1.4142136;
  ref_current = (power_factor * sin(spll1.theta[0] + PI * ratio)) *
                CURRENT_PEAK * 1.4142136;

  if (flag == 1) {
    // if (current_soft_start > 1)
    //   current_soft_start = 1;
    // current_soft_start += 0.00001;
    // ref_current = (power_factor * sin(spll1.theta[0] + PI / 6.0) * 1) *
    // CURRENT_PEAK *
    //               1.4142136 * current_soft_start;
    // general_graph[general_index++] = ref_current;
    // if (general_index >= GERERNAL_GRAPH_INDEX)
    //   general_index = 0;

    V_dc_feedback = rectifier_voltage;
    if (V_dc_feedback < 1)
      V_dc_feedback = 1;
  }
  V_in_feedback = spll1.osg_u[0];

  error = ref_current - grid_current;
  general_graph[general_index++] = V_in_feedback;
  if (general_index >= GERERNAL_GRAPH_INDEX)
    general_index = 0;
  output = (error * Kp_set + V_in_feedback * 30) /
           V_dc_feedback; // A sine wave, should between -1 and 1

  // Test rectifier
  // output = sin(spll1.theta[0]) * ratio;

  output_graph[output_index++] = output;
  if (output_index > OUTPUT_GRAPH_INDEX)
    output_index = 0;

  // output = sin(spll1.theta[0]) * 0.9;

  if (flag == 1) {
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;
    if (output > 1)
      output = 1;
    if (output < -1)
      output = -1;
    compare1 = (Uint16)(output * MAX_CMPA / 2.0 + 2250);
    compare1 = (Uint16)(output * MAX_CMPA / 2.0 + 2250);
    EPwm7Regs.CMPA.half.CMPA = (Uint16)compare1;
    EPwm8Regs.CMPA.half.CMPA = (Uint16)compare1;


    // EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    // EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    // // Set actions for rectifier
    // EPwm7Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    // EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    // EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
    // // Set actions for rectifier
    // EPwm8Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    // EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    // EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;
    // compare1 = (Uint16)(output * MAX_CMPA);
    // compare2 = (Uint16)(-1 * output * MAX_CMPA);
    // EPwm7Regs.CMPA.half.CMPA = (Uint16)compare1;
    // EPwm8Regs.CMPA.half.CMPA = (Uint16)compare2;
  }

  if (scope_mode == 0) {
    dutyCycle = (sin(spll1.theta[0]) + 1.0) / 2.0 * MAX_CMPA;
  }
  
  if (scope_mode == 1) {
    // Check
    dutyCycle = output * MAX_CMPA / 2 + 2250;
  }

  EPwm2Regs.CMPA.half.CMPA = (Uint16)dutyCycle;

  // Check SOGI
  // dutyCycle2 = (sin(spll1.theta[0] + PI * ratio) + 1.0) / 2.0 * MAX_CMPA;
  // dutyCycle2 = output * MAX_CMPA / 2 + 2500;
  dutyCycle2 = V_in_feedback * MAX_CMPA / 2.0 + 2250;
  // dutyCycle2 = ref_current * MAX_CMPA / 2 + 2000;
  // dutyCycle2 = grid_voltage / 20 * MAX_CMPA;
  EPwm3Regs.CMPA.half.CMPA = (Uint16)dutyCycle2;

  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}
