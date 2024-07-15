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

// #pragma CODE_SECTION(TIM0_IRQn, "ramfuncs");

// extern Uint16 RamfuncsLoadStart;
// extern Uint16 RamfuncsLoadEnd;
// extern Uint16 RamfuncsRunStart;
// extern Uint16 RamfuncsLoadSize;

// #define Kp 22.7089
#define Kp 25.7089
// #define Ki 1063.14
#define Ki 100.0
#define GRID_VPP 25
#define ISR_FREQUENCY 20000
#define V_DC_REFERENCE 25

Uint8 scope_mode;
float CURRENT_PEAK = 1;
_Bool flag_rectifier = 0;
_Bool flag_inverter = 0;
Uint8 K_RLC = 1;
float ratio = 0.5;

#define GERERNAL_GRAPH_INDEX 150
float general_graph[GERERNAL_GRAPH_INDEX];
Uint16 general_index;

#define THETA_GRAPH_INDEX 100
#define RECTIFIER_CURR_INDEX 150
#define OUTPUT_GRAPH_INDEX 150
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
float error2;
float error_before = 0;
float Ii_circle_p = 0;
float output = 0;
float output1 = 0;
float output2;
float V_mod2;

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

typedef struct {
  float kp, ki, kd;
  float error, lastError, prevError;
  float integral, maxIntegral;
  float output, maxOutput;
} PID;
PID Inverter_current_loop;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);

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
  InitPWM5();
  InitPWM6();
  InitPWM7();
  InitPWM8();
  KEY_Init();
  EALLOW;
  // Enable all the ePWM at the same time
  SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1; // ePWM7
  SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1; // ePWM8
  SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1; // ePWM5
  SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1; // ePWM6
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

  PID_Init(&Inverter_current_loop, 0.05, 0.1, 0, 1000, 1);
  TIM0_Init(90, 50.5); // 10K

  while (1) {
    if (KEY_Read() != 0) {
      if (KEY_Read() == 1) {
        flag_rectifier = 1 - flag_rectifier;
        while (KEY_Read() == 1)
          ;
      }
      if (KEY_Read() == 2) {
        flag_inverter = 1 - flag_inverter;
        while (KEY_Read() == 2)
          ;
      }
      if (KEY_Read() == 3) {
        K_RLC = 0 - K_RLC;
        while (KEY_Read() == 4)
          ;
      }
      if (KEY_Read() == 4) {
        power_factor += 0.1;
        if (power_factor >= 1)
          power_factor = 1;
        while (KEY_Read() == 2)
          ;
      }
      if (KEY_Read() == 5) {
        power_factor -= 0.1;
        if (power_factor <= 0)
          power_factor = 0;
        while (KEY_Read() == 3)
          ;
      }
    }

    if (flag_rectifier == 0) {
      current_soft_start = 0;
      EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      if (sin(spll1.theta[0]) > 0) {
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
    if (flag_inverter == 0) {
      EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      if (sin(spll1.theta[0]) > 0) {
        // Set actions for rectifier
        EPwm5Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm5Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm5Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        // Set actions for rectifier
        EPwm6Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm6Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm6Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      }
      if (sin(spll1.theta[0]) < 0) {
        // Set actions for rectifier
        EPwm5Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm5Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm5Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        // Set actions for rectifier
        EPwm6Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        EPwm6Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        EPwm6Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      }
    }
  }
}

interrupt void TIM0_IRQn(void) {
  EALLOW;

  GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
  normalized_voltage = grid_voltage / 35;
  spll1.u[0] = normalized_voltage;
  SPLL_1ph_SOGI_F_FUNC(&spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);

  if (flag_rectifier == 1) {
    // Soft Start: Set reference current increase gradually
    current_soft_start += 0.0001;
    if (current_soft_start > 1)
      current_soft_start = 1;
    if (power_factor > 1)
      power_factor = 1;
    if (power_factor < 0.5)
      power_factor = 0.5;
    ref_current =
        (power_factor * sin(spll1.theta[0]) +
         sqrt(1 - power_factor * power_factor) * cos(spll1.theta[0]) * K_RLC) *
        CURRENT_PEAK * 1.4142136;
  }
  if (flag_inverter == 1){
    // Soft Start: Set reference current increase gradually
  }

  V_dc_feedback = rectifier_voltage;
  if (V_dc_feedback < 1)
    V_dc_feedback = 1;
  V_in_feedback = spll1.osg_u[0];

  /********************* Rectifier Current Loop **************************/
  // Incremental form
  error = ref_current - grid_current;
  Ii_circle_p = Kp_set * (error - error_before);
  output1 += Ii_circle_p;
  output = (output1 + V_in_feedback * 35) / V_dc_feedback;
  error_before = error;
  // Position form
  // output = (error * Kp_set + V_in_feedback * 35) /
  //          V_dc_feedback; // A sine wave, should between -1 and 1
  /********************* Rectifier Current Loop **************************/

  /********************* Inverter Current Loop **************************/
  PID_Calc(&Inverter_current_loop, V_DC_REFERENCE, rectifier_voltage);
  output2 = Inverter_current_loop.output;
  if (output2 < 0)
    output2 = 0;
  if (output2 > 1)
    output2 = 1;
  // V_mod2 = output2 * sin(spll1.theta[0]); // Modulation waveform
  V_mod2 = sin(spll1.theta[0]) * ratio;
  /********************* Inverter Current Loop **************************/

  if (flag_rectifier == 1) {
    /********************* Rectifier SPWM modulation ************************/
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    // Set actions for rectifier
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
    // Set actions for rectifier
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;
    compare1 = (Uint16)(output * MAX_CMPA);
    compare2 = (Uint16)(-1 * output * MAX_CMPA);
    EPwm7Regs.CMPA.half.CMPA = compare1;
    EPwm8Regs.CMPA.half.CMPA = compare2;
    /********************* Inverter SPWM modulation ************************/
  }
  if (flag_inverter == 1) {
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    // Set actions for rectifier
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
    // Set actions for rectifier
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
    compare1 = (Uint16)(V_mod2 * MAX_CMPA);
    compare2 = (Uint16)(-1 * V_mod2 * MAX_CMPA);
    EPwm5Regs.CMPA.half.CMPA = compare1;
    EPwm6Regs.CMPA.half.CMPA = compare2;
    /********************* Inverter SPWM modulation ************************/
  }

  if (scope_mode == 0) {
    dutyCycle = (sin(spll1.theta[0]) + 1.0) / 2.0 * MAX_CMPA;
  }

  if (scope_mode == 1) {
    // Check
    dutyCycle = V_mod2 * MAX_CMPA / 2.5 + 2250;
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

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut) {
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->maxIntegral = maxI;
  pid->maxOutput = maxOut;
  pid->error = 0;
  pid->lastError = 0;
  pid->integral = 0;
  pid->output = 0;
}

void PID_Calc(PID *pid, float reference, float feedback) {
  pid->lastError = pid->error;
  pid->error = reference - feedback;
  float dout = (pid->error - pid->lastError) * pid->kd;
  float pout = pid->error * pid->kp;
  pid->integral += pid->error * pid->ki;
  if (pid->integral > pid->maxIntegral)
    pid->integral = pid->maxIntegral;
  else if (pid->integral < -pid->maxIntegral)
    pid->integral = -pid->maxIntegral;
  pid->output = pout + dout + pid->integral;
  if (pid->output > pid->maxOutput)
    pid->output = pid->maxOutput;
  else if (pid->output < -pid->maxOutput)
    pid->output = -pid->maxOutput;
}
