/*
 * epwm.c
 *
 *  Created on: Mar 25, 2024
 *      Author: nov4ou
 */
#include "epwm.h"
#include "F2806x_Device.h"
#include "F2806x_EPwm_defines.h"

float rectifier_dutycycle = 0;
extern float ref_sinwave;
Uint16 sineValue1 = 0, sineValue2 = 0;
extern _Bool flag;
float i_ref_rt;

void InitPWM2() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1; // ePWM2
  EDIS;

  InitEPwm2Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // Allow each timer to be sync'ed
  EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm2Regs.TBPHS.half.TBPHS = 0;
  EPwm2Regs.TBCTR = 0x0000; // Clear counter
  EPwm2Regs.TBPRD = PWM_TBPRD;
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm2Regs.CMPA.half.CMPA = 0; // Set compare A value
  EPwm2Regs.CMPB = 0;           // Set Compare B value

  // Set actions
  EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM1A on Zero
  EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM1A on event A, up count
  EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  //    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  //    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  //    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  //    EPwm2Regs.DBRED = 10;
  //    EPwm2Regs.DBFED = 10;

  EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm2Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void InitPWM3() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1; // ePWM2
  EDIS;

  InitEPwm3Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // A3low each timer to be sync'ed
  EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm3Regs.TBPHS.half.TBPHS = 0;
  EPwm3Regs.TBCTR = 0x0000; // Clear counter
  EPwm3Regs.TBPRD = PWM_TBPRD;
  EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm3Regs.CMPA.half.CMPA = 0; // Set compare A value
  EPwm3Regs.CMPB = 0;           // Set Compare B value

  // Set actions
  EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM1A on Zero
  EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM1A on event A, up count
  EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  //    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  //    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  //    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  //    EPwm2Regs.DBRED = 10;
  //    EPwm2Regs.DBFED = 10;

  EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm3Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void InitPWM7() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1; // ePWM7
  EDIS;

  InitEPwm7Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // Allow each timer to be sync'ed
  EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm7Regs.TBPHS.half.TBPHS = 0;
  EPwm7Regs.TBCTR = 0x0000; // Clear counter
  EPwm7Regs.TBPRD = PWM_TBPRD;
  EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and count down
  EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm7Regs.CMPA.half.CMPA = MAX_CMPA / 2; // Set compare A value
  EPwm7Regs.CMPB = MAX_CMPA / 2;           // Set Compare B value

  // Set actions
  // EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
  EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;   // Clear PWM1A on event A, up count
  EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM on down count
  // EPwm7Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  // EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm7Regs.DBRED = 10;
  EPwm7Regs.DBFED = 10;

  EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm7Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK =
      0; // Disable ePWM7 clock and start at the same time.
  EDIS;
}

void InitPWM8() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1; // ePWM8
  EDIS;

  InitEPwm8Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // Allow each timer to be sync'ed
  EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm8Regs.TBPHS.half.TBPHS = 0;
  EPwm8Regs.TBCTR = 0x0000; // Clear counter
  EPwm8Regs.TBPRD = PWM_TBPRD;
  // EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up and count down
  EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and count down
  EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm8Regs.CMPA.half.CMPA = MAX_CMPA / 2; // Set compare A value
  EPwm8Regs.CMPB = MAX_CMPA / 2;           // Set Compare B value

  // Set actions
  EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM1A on Zero
  EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM1A on event A, up count
  EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // // Set actions test
  // // EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
  // EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;   // Clear PWM1A on event A, up count
  // EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM on down count
  // EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  // EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm8Regs.DBRED = 10;
  EPwm8Regs.DBFED = 10;

  EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm8Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm8Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK =
      0; // Disable ePWM8 clock and start at the same time.
  EDIS;
}

__interrupt void epwm7_timer_isr(void) {

  // static Uint16 index = 0;
  // static const float step = 2 * PI * SINE_FREQ / PWM_FREQ;
  // i_ref_rt = (1 + sin(step * index)) * 10;
  // index++;
  // if (index >= (PWM_FREQ / SINE_FREQ)) {
  //   index = 0;
  // }

  // if (flag == 1) {
    // EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    //                                            // Set actions

    // // EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
    // // EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

    // if (ref_sinwave > 0) {
    //   EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET; // Set PWM1A on Zero
    //   EPwm8Regs.AQCTLA.bit.CAU = AQ_SET; // Clear PWM1A on event A, up count
    //   sineValue1 = (Uint16)(ref_sinwave * MAX_CMPA);
    //   EPwm7Regs.CMPA.half.CMPA = sineValue1;
    //   EPwm8Regs.CMPA.half.CMPA = MAX_CMPA;
    // }
    // if (ref_sinwave <= 0) {
    //   EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // Set PWM1A on Zero
    //   EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM1A on event A, up count
    //   sineValue2 = (Uint16)(MAX_CMPA - (-1 * ref_sinwave * MAX_CMPA));
    //   EPwm7Regs.CMPA.half.CMPA = sineValue2;
    //   EPwm8Regs.CMPA.half.CMPA = 0;
    // }
  // }

  //
  // Clear INT flag for this timer
  //
  EPwm7Regs.ETCLR.bit.INT = 1;

  //
  // Acknowledge this interrupt to receive more interrupts from group 3
  //
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// __interrupt void epwm8_timer_isr(void) {

//   // Set Compare values
//   EPwm8Regs.CMPA.half.CMPA = rectifier_dutycycle; // Set compare A value
//   EPwm8Regs.CMPB = rectifier_dutycycle;           // Set Compare B value

//   //
//   // Clear INT flag for this timer
//   //
//   EPwm8Regs.ETCLR.bit.INT = 1;

//   //
//   // Acknowledge this interrupt to receive more interrupts from group 3
//   //
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
// }
