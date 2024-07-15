#include "adc.h"
#include "F2806x_Device.h"
#include "epwm.h"

#define GRID_V_INDEX 100
#define RECTIFIER_CURR_INDEX 150

#define GRID_CURRENT_GRAPH 150

Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];
Uint16 Voltage3[10];

Uint8 index = 0;
Uint16 grid_curr_index;

float Vol1 = 0;
float Vol2 = 0;
float Vol3 = 0;
extern float grid_voltage;
extern float grid_current;
float rectifier_voltage;
float grid_vol_graph[GRID_V_INDEX];
float rectifier_volt_graph[RECTIFIER_CURR_INDEX];
float grid_current_graph[GRID_CURRENT_GRAPH];
Uint16 rectifier_volt_index;
Uint16 gridvindex;
float filtered_current;

typedef struct {
  float x_est; // Estimated state value
  float P_est; // Estimated state covariance
  float Q;     // Process noise covariance
  float R;     // Measurement noise covariance
} KalmanFilter;
KalmanFilter filtered_vol3;

void kalmanFilter_Init(KalmanFilter *kf);
float kalman_filter(KalmanFilter *kf, float z);

void ADC_Init() {

  kalmanFilter_Init(&filtered_vol3);
  //
  // Configure ADC
  //
  EALLOW;
  AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode

  //
  // ADCINT1 trips after AdcResults latch
  //
  AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

  AdcRegs.INTSEL1N2.bit.INT1E = 1;    // Enabled ADCINT1
  AdcRegs.INTSEL1N2.bit.INT1CONT = 0; // Disable ADCINT1 Continuous mode

  //
  // setup EOC1 to trigger ADCINT1 to fire
  //
  AdcRegs.INTSEL1N2.bit.INT1SEL = 1;

  AdcRegs.ADCSOC0CTL.bit.CHSEL =
      5; // set SOC0 channel select to ADCINA3     Bus Voltage
  AdcRegs.ADCSOC1CTL.bit.CHSEL =
      3; // set SOC1 channel select to ADCINA2     Grid Voltage
  AdcRegs.ADCSOC2CTL.bit.CHSEL =
      0xA; // set SOC1 channel select to ADCINA2   Grid Current
  //
  // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
  // first then SOC1
  //
  AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  //
  // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts
  // first then SOC1
  //
  AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

  //
  // set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts
  // first then SOC1
  //
  AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

  //
  // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
  //
  AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;

  //
  // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
  //
  AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;
  EDIS;

  //
  // Assumes ePWM1 clock is already enabled in InitSysCtrl();
  //
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOC on A group
  EPwm1Regs.ETSEL.bit.SOCASEL = 4;   // Select SOC from CMPA on upcount
  EPwm1Regs.ETPS.bit.SOCAPRD = 1;    // Generate pulse on 1st event
  EPwm1Regs.CMPA.half.CMPA = 0x0080; // Set compare A value
  EPwm1Regs.TBPRD = 0x5000;          // Set period for ePWM1
  // EPwm1Regs.TBPRD             = 4500;   // Set period for ePWM1
  EPwm1Regs.TBCTL.bit.CTRMODE = 0; // count up and start
}

//
// adc_isr -
//
__interrupt void adc_isr(void) {
  Voltage1[ConversionCount] = AdcResult.ADCRESULT0;
  Voltage2[ConversionCount] = AdcResult.ADCRESULT1;
  Voltage3[ConversionCount] = AdcResult.ADCRESULT2;

  Vol1 = Voltage1[ConversionCount] * 3.3 / 4095;
  Vol2 = Voltage2[ConversionCount] * 3.3 / 4095;
  Vol3 = Voltage3[ConversionCount] * 3.3 / 4095;
  // filtered_current = kalman_filter(&filtered_vol3,
  // Voltage3[ConversionCount]);

  rectifier_voltage = (Vol1 - 1.502) * 41.61;
  rectifier_volt_graph[rectifier_volt_index++] = rectifier_voltage;
  if (rectifier_volt_index > GRID_V_INDEX)
    rectifier_volt_index = 0;

  grid_voltage = Voltage2[ConversionCount] * 0.0272 - 50.498;
  grid_vol_graph[gridvindex++] = grid_voltage;
  if (gridvindex > GRID_V_INDEX)
    gridvindex = 0;

  grid_current = (Voltage3[ConversionCount] * 0.0016 - 2.9816) * 2;
  // grid_current = (Vol3 - 1.509) * 39.518 / 10;
  // grid_current = (Vol3 - 1.509) * 39.518 / 20;
  // filtered_current = kalman_filter(&filtered_vol3, Vol3);

  grid_current_graph[grid_curr_index++] = grid_current;
  if (grid_curr_index >= GRID_CURRENT_GRAPH)
    grid_curr_index = 0;

  //
  // If 20 conversions have been logged, start over
  //
  if (ConversionCount == 9) {
    ConversionCount = 0;
  } else {
    ConversionCount++;
  }

  //
  // Clear ADCINT1 flag reinitialize for next SOC
  //
  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE

  LoopCount++;
  return;
}

void kalmanFilter_Init(KalmanFilter *kf) {
  // Initialize Kalman Filter
  kf->x_est = 0;  // Initial estimate of state value
  kf->P_est = 1;  // Initial estimate of state covariance
  kf->Q = 0.0001; // Process noise covariance
  kf->R = 0.01;   // Measurement noise covariance
}

float kalman_filter(KalmanFilter *kf, float z) {
  // Prediction step
  float x_pred = kf->x_est;
  float P_pred = kf->P_est + kf->Q;

  // Update step
  float K = P_pred / (P_pred + kf->R);
  kf->x_est = x_pred + K * (z - x_pred);
  kf->P_est = (1 - K) * P_pred;

  return kf->x_est;
}
