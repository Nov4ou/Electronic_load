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
#include "oled.h"
#include "spi.h"
#include "timer.h"

#pragma CODE_SECTION(TIM0_IRQn, "ramfuncs");
#pragma CODE_SECTION(PID_Calc, "ramfuncs");

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

// #define Kp 22.7089
#define Kp 50.7089
// #define Ki 1063.14
#define Ki 5000.0
// #define ISR_FREQUENCY 20000
#define ISR_FREQUENCY 10000
#define V_DC_REFERENCE 50

#define WINDOW_SIZE 5 // Size of the moving average window
float buffer[WINDOW_SIZE];
int index_buffer = 0;
float filtered_output3;

Uint8 str[10];
Uint16 counter = 0;

// Initialize the buffer
void init_buffer() {
  Uint8 i = 0;
  for (i = 0; i < WINDOW_SIZE; i++) {
    buffer[i] = 0;
  }
}

// Add new data and calculate the moving average
float moving_average(float new_value) {
  Uint8 i = 0;
  buffer[index_buffer] = new_value;
  index_buffer = (index_buffer + 1) % WINDOW_SIZE;
  float sum = 0;
  for (i = 0; i < WINDOW_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / WINDOW_SIZE;
}

Uint8 scope_mode = 0;
float CURRENT_PEAK = 2;
_Bool flag_rectifier = 0;
_Bool flag_inverter = 0;
Int8 K_RLC = 1;
float ratio = 0.27;

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
float Kp_set = -12;
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
float error_before2 = 0;
float Ii_circle_p = 0;
float Ii_circle_p2 = 0;
float output = 0;
float output1 = 0;
float output2 = 0;
float output3 = 0;
float output3_2 = 0;
float last_output3 = 0;
float V_mod_inverter;

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
float deadband_56 = 10;
float deadband_78 = 10;

extern float rectifier_voltage;
extern float grid_inverter_current;

SPLL_1ph_SOGI_F spll1;
SPLL_1ph_SOGI_F spll2;

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
PID Inverter_voltage_loop, Inverter_current_loop, Inverter_grid_current_loop;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);

void OLED_Update();
void ftoa(float f, int precision);

int main() {
  memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
  InitFlash();
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
  // EPWM1_Init(MAX_CMPA);
  EPWM2_Init(MAX_CMPA);
  EPWM3_Init(MAX_CMPA);
  EPWM5_Init(MAX_CMPA);
  EPWM6_Init(MAX_CMPA);
  EPWM7_Init(MAX_CMPA);
  EPWM8_Init(MAX_CMPA);
  EPwm1Regs.TBCTL.bit.SWFSYNC = 1; // Master
  EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm5Regs.DBRED = deadband_56;
  EPwm5Regs.DBFED = deadband_56;
  EPwm6Regs.DBRED = deadband_56;
  EPwm6Regs.DBFED = deadband_56;
  // Set actions for rectifier
  EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
  // Set actions for rectifier
  EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

  EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm7Regs.DBRED = deadband_56;
  EPwm7Regs.DBFED = deadband_56;
  EPwm8Regs.DBRED = deadband_56;
  EPwm8Regs.DBFED = deadband_56;
  // Set actions for rectifier
  EPwm7Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
  // Set actions for rectifier
  EPwm8Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;

  KEY_Init();
  SPIB_Init();
  OLED_Init();
  OLED_Clear();

  // EALLOW;
  // // Enable all the ePWM at the same time
  // SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1; // ePWM7
  // SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1; // ePWM8
  // SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1; // ePWM5
  // SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1; // ePWM6
  // EDIS;

  IER |= M_INT3;
  PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx8 = 1;
  EINT; // Enable Global interrupt INTM
  ERTM; // Enable Global realtime interrupt DBGM

  SPLL_1ph_SOGI_F_init(GRID_FREQ, ((float)(1.0 / ISR_FREQUENCY)), &spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);
  spll1.osg_coeff.osg_k = 1.0;
  SPLL_1ph_SOGI_F_init(GRID_FREQ, ((float)(1.0 / ISR_FREQUENCY)), &spll2);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll2);
  spll2.osg_coeff.osg_k = 1.414;
  spll1.lpf_coeff.B0_lf = (float32)((2 * Kp + Ki / ISR_FREQUENCY) / 2);
  spll1.lpf_coeff.B1_lf = (float32)(-(2 * Kp - Ki / ISR_FREQUENCY) / 2);
  spll2.lpf_coeff.B0_lf = (float32)((2 * Kp + Ki / ISR_FREQUENCY) / 2);
  spll2.lpf_coeff.B1_lf = (float32)(-(2 * Kp - Ki / ISR_FREQUENCY) / 2);

  // Only Voltage PI Loop
  PID_Init(&Inverter_voltage_loop, 0.0001, 0.001, 0, 10, 5);
  PID_Init(&Inverter_current_loop, 0.01, 0.001, 0, 25, 15);
  PID_Init(&Inverter_grid_current_loop, 7, 0.001, 0, 10, 100);

  // Dual PI Loop
  // PID_Init(&Inverter_voltage_loop, 0.0001, 0.001, 0, 5, 1);
  // PID_Init(&Inverter_current_loop, 30, 0, 0, 30, 12.5);

  // TIM0_Init(90, 50.5); // 20K
  TIM0_Init(90, 101); // 10K

  while (1) {
    OLED_Update();
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
      if (KEY_Read() == 4) {
        K_RLC = 0 - K_RLC;
        while (KEY_Read() == 4)
          ;
      }
      if (KEY_Read() == 5) {
        power_factor += 0.1;
        if (power_factor >= 1)
          power_factor = 1;
        while (KEY_Read() == 5)
          ;
      }
      if (KEY_Read() == 6) {
        power_factor -= 0.1;
        if (power_factor <= 0.5)
          power_factor = 0.5;
        while (KEY_Read() == 6)
          ;
      }
    }

    if (flag_rectifier == 0) {
      current_soft_start = 0;
      EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      // Set actions for rectifier
      EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
      EPwm7Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
      EPwm7Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      // Set actions for rectifier
      EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
      EPwm8Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
      EPwm8Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
    }
    if (flag_inverter == 0) {
      EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
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

interrupt void TIM0_IRQn(void) {
  EALLOW;

  GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
  normalized_voltage = grid_voltage / 50;
  spll1.u[0] = normalized_voltage;
  SPLL_1ph_SOGI_F_FUNC(&spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);
  spll1.osg_coeff.osg_k = 1.0;

  if (flag_rectifier == 1) {
    // Soft Start: Set reference current increase gradually
    current_soft_start += 0.0001;
    if (current_soft_start >= 1)
      current_soft_start = 1;
    if (power_factor > 1)
      power_factor = 1;
    if (power_factor < 0.5)
      power_factor = 0.5;
    ref_current = (power_factor * sin(spll1.theta[0] + 0.1) +
                   sqrt(1 - power_factor * power_factor) *
                       cos(spll1.theta[0] + 0.1) * K_RLC) *
                  CURRENT_PEAK * 1.4142136 * current_soft_start;

    // ref_current = sin(spll1.theta[0] + 0.1) * CURRENT_PEAK * 1.4142136 *
    //               current_soft_start;
  }

  V_dc_feedback = rectifier_voltage;
  if (V_dc_feedback < 1)
    V_dc_feedback = 1;
  V_in_feedback = spll1.osg_u[0];

  /********************* Rectifier Current Loop **************************/
  // Incremental form
  error = ref_current - grid_current;
  // spll2.u[0] = error / 20;
  // SPLL_1ph_SOGI_F_FUNC(&spll2);
  // SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
  //                              (float)(2 * PI * GRID_FREQ), &spll2);
  // spll2.osg_coeff.osg_k = 1.414;
  Ii_circle_p = Kp_set * (error - error_before);
  output1 += Ii_circle_p;
  // output = (output1 + spll2.osg_u[0] * 10 * -80 + V_in_feedback * 45 * ratio)
  // /
  //          V_dc_feedback;
  output = (output1 + V_in_feedback * 50) / V_DC_REFERENCE;

  // output = (output1 + spll2.osg_u[0] * 20 * -80 + V_in_feedback * 50) /
  // V_DC_REFERENCE; output = (output1 + V_in_feedback * 50) / V_DC_REFERENCE;

  // output = 0.4 * sin(spll1.theta[0] + 0.1);

  error_before = error;
  // Position form
  // output = (error * Kp_set + V_in_feedback * 35) /
  //          V_dc_feedback; // A sine wave, should between -1 and 1
  /********************* Rectifier Current Loop **************************/

  /********************* Inverter Current Loop **************************/

  if (flag_inverter == 1) {
    /************************ Dual PI Loop **************************/
    PID_Calc(&Inverter_current_loop, rectifier_voltage, V_DC_REFERENCE);
    output2 = 0 + Inverter_current_loop.output;
    if (output2 < 0)
      output2 = 0;
    if (output2 > 5 * 1.4142136)
      output2 = 5 * 1.4142136;

    // output3 =
    //     5 * (grid_inverter_current + output2 * sin(spll1.theta[0] + 0.1)) +
    //     V_in_feedback * 50 * ratio;

    // output3 =
    //     9 * (output2 * sin(spll1.theta[0] + 0.1) - grid_inverter_current) +
    //     V_in_feedback * 50 * 0.5;

    output2 = output2 * sin(spll1.theta[0] + 0.1);
    PID_Calc(&Inverter_grid_current_loop, output2, grid_inverter_current);
    output3 = Inverter_grid_current_loop.output + V_in_feedback * 50;


    V_mod_inverter = output3 / V_DC_REFERENCE; // Modulation waveforme
    /************************ Dual PI Loop **************************/

    /************************ Only Voltage PI Loop **************************/
    // PID_Calc(&Inverter_voltage_loop, rectifier_voltage, V_DC_REFERENCE);
    // output3 = 0 + Inverter_voltage_loop.output;
    // if (output3 < 0)
    //   output3 = 0;
    // if (output3 > 1)
    //   output3 = 1;
    // V_mod_inverter = output3 * sin(spll1.theta[0] + 0.1);
    /************************ Only Voltage PI Loop **************************/

    // Open Loop Test
    // V_mod_inverter = sin(spll1.theta[0] + 0.1) * ratio;
  }

  /********************* Inverter Current Loop **************************/

  if (flag_rectifier == 1) {
    /********************* Rectifier SPWM modulation ************************/
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm7Regs.DBRED = deadband_78;
    EPwm7Regs.DBFED = deadband_78;
    EPwm8Regs.DBRED = deadband_78;
    EPwm8Regs.DBFED = deadband_78;
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
    /********************* Rectifier SPWM modulation ************************/
  }
  if (flag_inverter == 1) {
    /********************* Inverter SPWM modulation ************************/
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBRED = deadband_56;
    EPwm5Regs.DBFED = deadband_56;
    EPwm6Regs.DBRED = deadband_56;
    EPwm6Regs.DBFED = deadband_56;
    // Set actions for rectifier
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
    // Set actions for rectifier
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
    compare1 = (Uint16)(V_mod_inverter * MAX_CMPA);
    compare2 = (Uint16)(-1 * V_mod_inverter * MAX_CMPA);
    EPwm5Regs.CMPA.half.CMPA = compare1;
    EPwm6Regs.CMPA.half.CMPA = compare2;
    /********************* Inverter SPWM modulation ************************/
  }

  if (scope_mode == 0) {
    dutyCycle = V_mod_inverter * MAX_CMPA / 2.0 + MAX_CMPA / 2;
  }

  if (scope_mode == 1) {
    // Check
    dutyCycle = output * MAX_CMPA / 2.0 + MAX_CMPA / 2;
  }

  EPwm2Regs.CMPA.half.CMPA = (Uint16)dutyCycle;

  // Check SOGI
  dutyCycle2 = sin(spll1.theta[0] + 0.1) / 2.0 * MAX_CMPA + MAX_CMPA / 2.0;
  //  dutyCycle2 = output * MAX_CMPA / 2 + 2500;
  // dutyCycle2 = V_mod_inverter * MAX_CMPA / 2.0 + 2250;
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

void OLED_Update() {
  counter++;
  if (counter == 50000) {
    OLED_ShowString(0, 0, "Inverter: ", 16);
    if (flag_inverter == 1) {
      OLED_ShowString(75, 0, "    1", 16);
    } else {
      OLED_ShowString(75, 0, "    0", 16);
    }

    OLED_ShowString(0, 2, "Rectifier: ", 16);
    if (flag_rectifier == 1) {
      OLED_ShowString(83, 2, "   1", 16);
    } else {
      OLED_ShowString(83, 2, "   0", 16);
    }

    OLED_ShowString(0, 4, "K_RLC: ", 8);
    if (K_RLC == 1) {
      OLED_ShowString(75, 4, "     1", 8);
    } else {
      OLED_ShowString(75, 4, "    -1", 8);
    }

    OLED_ShowString(0, 5, "Power factor: ", 8);
    ftoa(power_factor, 1);
    OLED_ShowString(95, 6, str, 4);

    // OLED_ShowString(0, 2, "Eff:", 16);
    // if (Vol1Loop == 1 && Vol2Loop == 1) {
    //   ftoa(filteredEff * 100, 1);
    //   OLED_ShowString(45, 2, str, 16);
    // }

    // OLED_ShowString(0, 4, "Loop: ", 16);
    // if (Vol1Loop == 1 && Vol2Loop == 1)
    //   OLED_ShowString(45, 4, "ON ", 16);
    // else
    //   OLED_ShowString(45, 4, "OFF", 16);

    counter = 0;
  }
}

// This function converts a float to a string with a specified precision.
void ftoa(float f, int precision) {
  int j;

  // Split the float into integer and fractional parts
  int int_part = (int)f;
  float frac_part = f - int_part;

  int i = 0;
  // If the integer part is 0, add '0' to the string
  if (int_part == 0) {
    str[i++] = '0';
  } else {
    // Convert the integer part to string
    while (int_part) {
      str[i++] = int_part % 10 + '0';
      int_part /= 10;
    }
  }

  // Reverse the string to get the correct order
  for (j = 0; j < i / 2; j++) {
    char temp = str[j];
    str[j] = str[i - j - 1];
    str[i - j - 1] = temp;
  }

  // Add the decimal point
  str[i++] = '.';

  // Convert the fractional part to string with the specified precision
  for (j = 0; j < precision; j++) {
    frac_part *= 10;
    int digit = (int)frac_part;
    str[i++] = digit + '0';
    frac_part -= digit;
  }

  // Terminate the string
  str[i] = '\0';
}
