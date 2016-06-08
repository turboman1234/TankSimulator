#include "initPeripheral.h"
#include "userLibrary.h"
#include "definitions.h"
#include "tankSimulator.h"

Tank TankValues;


void InitPeripheral(void)
{
    InitADC(PUMP_CONTROL_VOLTAGE);
    InitDAC(FLUID_LEVEL, 0);
    InitDAC(OUTPUT_FLOW, 0);
    
    InitLED(LED_2_CM);
    InitLED(LED_4_CM);
    InitLED(LED_6_CM);
    InitLED(LED_8_CM);
    InitLED(LED_10_CM);
    InitLED(ALARM_LED);
}

void InitInitialCondition(void)
{
    TankValues.currentControlVoltage = 0.0;
    TankValues.oldControlVoltage = 0.0;
    TankValues.currentFluidLevel = 0.0;
    TankValues.oldFluidLevel = 0.0;
    TankValues.outputFlow = 0.0;
    
    SetLED(LED_2_CM, OFF);
    SetLED(LED_4_CM, OFF);
    SetLED(LED_6_CM, OFF);
    SetLED(LED_8_CM, OFF);
    SetLED(LED_10_CM, OFF);
    SetLED(ALARM_LED, OFF);
}

void ReadControlSignal(void)
{
    int adcCode;
    
    adcCode = GetAnalogInput(PUMP_CONTROL_VOLTAGE);
    
    // normalize adcCode in range 0-3999
    if(adcCode < 96)                            // adc code in range 0 - 96 may be produced by noise and that is why it is not good to use it
    {
        adcCode = 0;
    }
    else
    {
        adcCode = adcCode - 96;
    }
    
    // actualize u(k-1)
    TankValues.oldControlVoltage = TankValues.currentControlVoltage;
    
    // actualize u(k)
    TankValues.currentControlVoltage = adcCode * ADC_CODE_TO_VOLTAGE_CONSTANT;
}

void CalculateTankLevel(void)
{
    // h(k) = h(k-1) - T0*A*sqrt(h(k-1)) + T0*B*u(k-1) - discretized equation
    TankValues.currentFluidLevel = TankValues.oldFluidLevel - (T0 * A * sqrt(TankValues.oldFluidLevel)) + B * T0 * TankValues.oldControlVoltage;
    
    // F_out(k) = So * sqrt(2 * g * h(k))
    TankValues.outputFlow = S_OUT * sqrt(2 * GRAVITY_CONSTANT * TankValues.currentFluidLevel);
}

void SetFluidLevelValueToOutput(void)
{
    int dacCode;
    
    //normalization - 0.0, m ----> 0, 0.10098, m - 4095 => in this way the DAC will realize 0.1, m + 0.098,mm or ~0.1% overfilling
    dacCode = (int)round((TankValues.currentFluidLevel * FLUID_LEVEL_TO_DAC_CODE) + 96);
    
    if(dacCode > 4095)
    {
        // Overfilling
        dacCode = 4095;
        
        SetLED(ALARM_LED, ON);
    }
    else
    {
        SetLED(ALARM_LED, OFF);
    }
    
    SetAnalogOutput(FLUID_LEVEL, dacCode);
}
