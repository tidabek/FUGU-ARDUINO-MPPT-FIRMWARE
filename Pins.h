#pragma once

// ============================================================================
// HARDWARE PIN DEFINITIONS: VIETNAMESE PCB (4EVN MPPT V2)
// ============================================================================

// Drivers & Switching Stage
#define buck_IN           33   // Buck MOSFET Driver PWM Pin
#define buck_EN           32   // Buck MOSFET Driver Enable Pin
#define backflow_MOSFET   27   // Backflow MOSFET



// Peripherals & Control Lines
#define Load              12   // Load MOSFET Control
#define FAN               16   // Cooling Fan MOSFET Control
#define LED               2    // Onboard Status LED
#define ADC_ALERT         34   // ADC Alert
#define TempSensor        35   // NTC Temperature Sensor

// Buttons
#define buttonLeft        18   // Left Button
#define buttonRight       17   // Right Button
#define buttonBack        19   // Back Button
#define buttonSelect      23   // Select Button

// ADS1015 / ADS1115 ADC Channels
#define ADS_SOLAR_V_CHAN   3   // Channel A2: Solar Input Voltage (VSI)
#define ADS_BATT_V_CHAN    1   // Channel A1: Battery Output Voltage (VSO)
#define ADS_CURRENT_CHAN   2   // Channel A3: Current Sensor (CSI)