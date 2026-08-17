#include "Pins.h"
extern unsigned long prevLCDMillis;
const unsigned long MIN_ABS_TIME = 30UL * 60UL * 1000UL;   // 30 minutes
const unsigned long SLEEP_DELAY_MS = 3UL * 60UL * 1000UL; // 3 minutes at < 0.3A before sleeping
const float CV_HYSTERESIS = 0.02;
float tailCurrentThresh = 2.00;
float tailCurrentPercent = 0.02;
unsigned long sleepStartMillis = 0;

void buck_Enable(){                                                                  
  buckEnable = 1;
  digitalWrite(buck_EN,HIGH);
  digitalWrite(LED,HIGH);
}

void buck_Disable(){                                                                 
  buckEnable = 0;
  digitalWrite(buck_EN,LOW);
  digitalWrite(LED,LOW);
  PWM = 0;
}   

void predictivePWM(){                                                                
  if(voltageInput <= 0){ PPWM = 0; }                      
  else{ PPWM = (PPWM_margin * pwmMax * voltageOutput) / (100.00 * voltageInput); }              
  PPWM = constrain(PPWM, 0, pwmMaxLimited);
}   

void PWM_Modulation(){
  predictivePWM();                                                   
    if(chargingState == 0 && currentOutput < currentCharging && voltageOutput < voltageBatteryMax) {
    PWM = constrain(PWM, PPWM, pwmMaxLimited);
  } else {
    PWM = constrain(PWM, 0, pwmMaxLimited);
  }
  if (PWM <= 0) {
    buck_Disable(); 
    ledcWrite(buck_IN, 0);
  } else {
    ledcWrite(buck_IN, PWM);
    buck_Enable(); 
  }
}
     
void updateBatteryProfile(){
    switch (battPreset){
        // LiFePO4
        case 0:
            tailCurrentPercent = 0.05f;                 // 5%
            absWindow = 1UL * 60UL * 1000UL;            // 1 minute
            break;
        // Lithium-Ion
        case 1:
            tailCurrentPercent = 0.05f;                 // 5%
            absWindow = 5UL * 60UL * 1000UL;            // 5 minutes
            break;
        // AGM / Sealed
        case 2:
            tailCurrentPercent = 0.02f;                 // 2%
            absWindow = 120UL * 60UL * 1000UL;          // 2 hours
            break;
        // Flooded Lead Acid
        case 3:
            tailCurrentPercent = 0.015f;                // 1.5%
            absWindow = 180UL * 60UL * 1000UL;          // 3 hours
            break;
        // Custom
        case 4:
            // Keep user configured timer
            if(tailCurrentPercent <= 0.0f)
                tailCurrentPercent = 0.02f;
            break;
    }
    tailCurrentThresh = batteryCapacityAH * tailCurrentPercent;
}

void Charging_Algorithm(){
  
  if(ERR > 0 || chargingPause == 1){ buck_Disable(); }               // ERROR PRESENT - Turn off MPPT buck 
  else{
    if(REC == 1){                                                    // IUV RECOVERY 
      REC = 0;
      chargingState = 0;                                             // Reset charging state back to BULK/MPPT
      buck_Disable();
      Serial.println("> Solar Panel Detected");
      Serial.print("> Computing For Predictive PWM ");
      for(int i = 0; i < 40; i++){ Serial.print("."); delay(30); }                        
      Serial.println("");
      Read_Sensors();
      predictivePWM();
      PWM = PPWM; 
      lcd.clear();
      prevLCDMillis = 0;
    }  
    else{            
      // ================= MPPT & MULTI-STAGE CHARGING ALGORITHM ================= //
      
      // --- 1. STATE TRANSITION LOGIC ---
      float rechargeVoltage;
      float standbyVoltage;

      if (battPreset == 0 || battPreset == 1){
        // Lithium Setup
        rechargeVoltage = voltageBatteryFloat;        // e.g., 13.2V - 13.4V
        standbyVoltage  = voltageBatteryFloat + 0.20; // Soft finish voltage (e.g., 13.6V)
      }
      else{
        // Lead Acid Setup
        rechargeVoltage = voltageBatteryFloat - 0.30;
        standbyVoltage  = voltageBatteryFloat;
      }

      // STAGE 0: BULK -> ABSORPTION
      if (chargingState == 0) { 
        if (voltageOutput >= (voltageBatteryMax - CV_HYSTERESIS)) {
          chargingState = 1;
          absStartMillis = millis();
        }
      }
      // STAGE 1: ABSORPTION -> FINISH / FLOAT
      else if (chargingState == 1) { 
        unsigned long absElapsed = millis() - absStartMillis;
        bool minTimeReached = (absElapsed >= MIN_ABS_TIME);
        bool maxTimeReached = (absElapsed >= absWindow);
        bool tailReached    = (currentOutput <= tailCurrentThresh) && (currentOutput > 0.05);

        if ((minTimeReached && tailReached) || maxTimeReached) {
          if (battPreset == 0 || battPreset == 1) {
            chargingState = 3; // LITHIUM: Skip float, go directly to SLEEP/OFF
          } else {
            chargingState = 2; // LEAD ACID: Move to Finish/Standby (Float) stage
          }
          sleepStartMillis = millis();
        }

        // Safety Dropback to Bulk
        if (voltageOutput < (voltageBatteryMax - 0.25)) {
          chargingState = 0;
        }
      }
      // STAGE 2: FINISH / STANDBY (LEAD-ACID ONLY)
      else if (chargingState == 2) {
        // Re-bulk trigger
        if (voltageOutput < rechargeVoltage) {
          chargingState = 0;
        }
      }
      // STAGE 3: SOFT SLEEP / TERMINATION
      else if (chargingState == 3) {
        // Smoothly step down PWM before full shutdown
        if (PWM > 0) {
          PWM--;
        }
        
        // Restart condition
        if (voltageOutput < rechargeVoltage) {
          chargingState = 0;
        }
      }

      // --- 2. STATE EXECUTION LOGIC ---
      if (chargingState == 0) {                                                    
        // BULK STAGE (MPPT TRACKING)
        if(currentOutput > currentCharging)                         {PWM--;}                           
        else if(voltageOutput > voltageBatteryMax)                  {PWM--;}                           
        else{                    
          if(powerInput > powerInputPrev && voltageInput > voltageInputPrev)        {PWM--;}  
          else if(powerInput > powerInputPrev && voltageInput < voltageInputPrev)   {PWM++;} 
          else if(powerInput < powerInputPrev && voltageInput > voltageInputPrev)   {PWM++;}  
          else if(powerInput < powerInputPrev && voltageInput < voltageInputPrev)   {PWM--;}  
          else if(voltageOutput < voltageBatteryMax)                                {PWM++;}  
        
          powerInputPrev   = powerInput;
          voltageInputPrev = voltageInput;
        }   
      }
      else if (chargingState == 1) {                                               
        // ABSORPTION STAGE
        if(currentOutput > currentCharging)                          {PWM--;}        
        else if(voltageOutput > voltageBatteryMax + CV_HYSTERESIS)   {PWM--;}                           
        else if(voltageOutput < voltageBatteryMax - CV_HYSTERESIS)   {PWM++;}                          
      }
      else if (chargingState == 2) {                                               
        // FINISH / STANDBY STAGE
        if(currentOutput > currentCharging)                          {PWM--;}  
        else if(voltageOutput > standbyVoltage + CV_HYSTERESIS)      {PWM--;}                   
        else if(voltageOutput < standbyVoltage - CV_HYSTERESIS)      {PWM++;}                         
      }
      else if (chargingState == 3) {
        // SLEEP STAGE
        // Handled by PWM ramp-down above and automatic buck_Disable() inside PWM_Modulation()
      }
      
      PWM_Modulation(); // Execute PWM Modulation & Driver Control                                                                      
    }  
  }
}