#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "initPeripheral.h"
#include "userLibrary.h"
#include "definitions.h"
#include "mytim.h"
#include "VTimer.h"
#include "LCD.h"
#include "tankSimulator.h"

Tank TankValues;

//LCD Data buffer
Tank LCDBuffer;
//Row arrays for more friendly writting on LCD
char Row1[ROW_LENGHT], Row2[ROW_LENGHT], Row3[ROW_LENGHT], Row4[ROW_LENGHT];

// Flags for handshaking between tank simulator and LCD
BOOL SimulatorAcknowledge;
BOOL SimulatorReady;
BOOL LCDAcknowledge;
BOOL LCDRequest;


void InitTankPeripheral(void)
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
    
    InitTIM5(SAMPLE_TIME);
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
    
    SimulatorAcknowledge = FALSE;
    SimulatorReady = FALSE;
    
    LCDAcknowledge = FALSE;
    LCDRequest = FALSE;
    
    // Start Tank simulator
    TIM_Cmd(TIM5, ENABLE);
}

void ReadControlSignal(void)
{
    int adcCode;
    
    adcCode = GetAnalogInput(PUMP_CONTROL_VOLTAGE);
    
    // normalize adcCode in range 95-4055
    if(adcCode < MIN_ADC_VALUE)                            // adc code in range 0 - 94 may be produced by noise and that is why it is not good to use it
    {
        adcCode = 0;
    }
    else
    {
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    // actualize u(k-1) = u(k)
    TankValues.oldControlVoltage = TankValues.currentControlVoltage;
    
    TankValues.readControlVoltage = (float)adcCode * ADC_CODE_TO_VOLTAGE_CONSTANT;
    
    // actualize u(k) - simulate pump dead zone
    if (TankValues.readControlVoltage < U_MIN)
    {
        // In the dead zone of the pump [0 - 3], V
        TankValues.currentControlVoltage = 0.0;
    }
    else if(TankValues.readControlVoltage > U_MAX)
    {
        // Over max voltage
        TankValues.currentControlVoltage = U_MAX - U_MIN;
        TankValues.readControlVoltage = U_MAX;
    }
    else
    {
        // In linear zone
        TankValues.currentControlVoltage = TankValues.readControlVoltage - U_MIN;
    }
}

void CalculateTankLevel(void)
{
    // h(k-1) = h(k)
    TankValues.oldFluidLevel = TankValues.currentFluidLevel;
    
    
    // h(k) = h(k-1) - T0*A*sqrt(h(k-1)) + T0*B*u(k-1) - discretized equation
    TankValues.currentFluidLevel = TankValues.oldFluidLevel - (T0 * A * sqrt(TankValues.oldFluidLevel)) + (B * T0 * TankValues.oldControlVoltage);
    
    // F_out(k) = So * sqrt(2 * g * h(k))
    TankValues.outputFlow = S_OUT * sqrt(2 * GRAVITY_CONSTANT * TankValues.currentFluidLevel);
    
    // level is decresing and become less then down sensor level
    if(TankValues.currentFluidLevel < TankValues.oldFluidLevel && TankValues.currentFluidLevel < FLUID_LEVEL_LOW_BORDER)
    {
        TankValues.currentFluidLevel = 0.0;
    }
    // level is incresing over 0.1001, m and is outside sensor's range
    else if(TankValues.currentFluidLevel > FLUID_LEVEL_HIGH_BORDER)
    {
        TankValues.currentFluidLevel = FLUID_LEVEL_HIGH_BORDER;
    }
}

void SetFluidLevelAndOutputFlowValuesToOutput(void)
{
    int dacCode;
    
    // Fluid level
    
    //normalization - 0.0, m ----> 0, 0.1001, m - 4055 => in this way the DAC will realize 0.1, m + 0.1,mm or 0.1% overfilling
    dacCode = (int)round((TankValues.currentFluidLevel * FLUID_LEVEL_TO_DAC_CODE) + (float)MIN_DAC_VALUE);
    
    if(dacCode > MAX_DAC_VALUE)
    {
        // Overfilling - fluid level is 0.1001 or bigger
        dacCode = MAX_DAC_VALUE;
    }
    
    SetAnalogOutput(FLUID_LEVEL, dacCode);
    
    dacCode = 0;
    // Output flow
    
    //normalization - 0.0, m^3/s ----> 0, F_OUT_MAX * 1.005, m^3/s - 4055 => in this way the DAC will realize min-max output flow
    dacCode = (int)round((TankValues.outputFlow * OUTPUT_FLOW_TO_DAC_CODE) + (float)MIN_DAC_VALUE);
    
    if(dacCode > MAX_DAC_VALUE)
    {
        // Output flow rate is over the maximum value
        dacCode = MAX_DAC_VALUE;
    }
    
    SetAnalogOutput(OUTPUT_FLOW, dacCode);
}

void SetLEDLevelIndicator(void)
{
    if(TankValues.currentFluidLevel == TANK_LEVEL_0_CM)
    {
        SetLED(LED_2_CM, OFF);
        SetLED(LED_4_CM, OFF);
        SetLED(LED_6_CM, OFF);
        SetLED(LED_8_CM, OFF);
        SetLED(LED_10_CM, OFF);
        SetLED(ALARM_LED, OFF);
    }
    else if(TankValues.currentFluidLevel <= TANK_LEVEL_2_CM)
    {
        SetLED(LED_2_CM, ON);
        SetLED(LED_4_CM, OFF);
        SetLED(LED_6_CM, OFF);
        SetLED(LED_8_CM, OFF);
        SetLED(LED_10_CM, OFF);
        SetLED(ALARM_LED, OFF);
    }
    else if(TankValues.currentFluidLevel <= TANK_LEVEL_4_CM)
    {
        SetLED(LED_2_CM, ON);
        SetLED(LED_4_CM, ON);
        SetLED(LED_6_CM, OFF);
        SetLED(LED_8_CM, OFF);
        SetLED(LED_10_CM, OFF);
        SetLED(ALARM_LED, OFF);
    }
    else if(TankValues.currentFluidLevel <= TANK_LEVEL_6_CM)
    {
        SetLED(LED_2_CM, ON);
        SetLED(LED_4_CM, ON);
        SetLED(LED_6_CM, ON);
        SetLED(LED_8_CM, OFF);
        SetLED(LED_10_CM, OFF);
        SetLED(ALARM_LED, OFF);
    }
    else if(TankValues.currentFluidLevel <= TANK_LEVEL_8_CM)
    {
        SetLED(LED_2_CM, ON);
        SetLED(LED_4_CM, ON);
        SetLED(LED_6_CM, ON);
        SetLED(LED_8_CM, ON);
        SetLED(LED_10_CM, OFF);
        SetLED(ALARM_LED, OFF);
    }
    else if(TankValues.currentFluidLevel <= TANK_LEVEL_10_CM)
    {
        SetLED(LED_2_CM, ON);
        SetLED(LED_4_CM, ON);
        SetLED(LED_6_CM, ON);
        SetLED(LED_8_CM, ON);
        SetLED(LED_10_CM, ON);
        SetLED(ALARM_LED, OFF);
    }
    else if(TankValues.currentFluidLevel > TANK_LEVEL_10_CM)
    {
        // Alarm
        SetLED(LED_2_CM, OFF);
        SetLED(LED_4_CM, OFF);
        SetLED(LED_6_CM, OFF);
        SetLED(LED_8_CM, OFF);
        SetLED(LED_10_CM, OFF);
        SetLED(ALARM_LED, ON);
    }
}

void TankSimulatorTask(void)
{
        ReadControlSignal();
        
        CalculateTankLevel();
        
        SetFluidLevelAndOutputFlowValuesToOutput();
        
        SetLEDLevelIndicator();
}

// This fuction realize discrete execution for Tank simulator
void TIM5_IRQHandler(void)
{
    TankSimulatorTask();
    
    if(LCDRequest == TRUE)
    {
        SimulatorAcknowledge = TRUE;
    }
    else if(SimulatorAcknowledge == TRUE)
    {
        // send process data to LCD
        LCDBuffer.currentFluidLevel = TankValues.currentFluidLevel * 100;       // cm
        LCDBuffer.currentControlVoltage = TankValues.currentControlVoltage;     // V
        LCDBuffer.readControlVoltage = TankValues.readControlVoltage;           // V
        LCDBuffer.outputFlow = TankValues.outputFlow * 1000000;                 //cm3/s
        
        SimulatorAcknowledge = FALSE;
        SimulatorReady = TRUE;
    }
    else if(LCDAcknowledge == TRUE)
    {
        SimulatorReady = FALSE;
    }
    else
    {
        // do nothing - idle
    }
    
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}


void SimulatorDisplayDataTask(void)
{
    char Buffer[81]; // (4 * row lenght) + 1 (for '\0')
    float Fin;
    
    if(IsVTimerElapsed(LCD_REFRESH_TIMER) == ELAPSED)
    {
        if(SimulatorAcknowledge == TRUE)
        {
            LCDRequest = FALSE;
        }
        else if(SimulatorReady == TRUE)
        {
            Fin = LCDBuffer.currentControlVoltage * PUMP_COEF * 1000000;                // cm3/s
            
            if(LCDBuffer.currentFluidLevel < (H_MAX * 100.0))
            {
                sprintf(Row1, "Fluid level: %2.2f,cm", LCDBuffer.currentFluidLevel);
            }
            else
            {
                sprintf(Row1, "Fluid level:%2.2f,cm", LCDBuffer.currentFluidLevel);
            }
            if(Fin > 9.99)
            {
                sprintf(Row2, "Fin:    %2.2f, cm3/s", Fin);
            }
            else
            {
                sprintf(Row2, "Fin:     %2.2f, cm3/s", Fin);
            }
            if(LCDBuffer.outputFlow > 9.99)
            {
                sprintf(Row3, "Fout:   %2.2f, cm3/s", LCDBuffer.outputFlow);
            }
            else
            {
                sprintf(Row3, "Fout:    %2.2f, cm3/s", LCDBuffer.outputFlow);
            }
            if(LCDBuffer.readControlVoltage >= U_MAX)
            {
                sprintf(Row4, "Ctrl volt:  %2.2f, V", LCDBuffer.readControlVoltage);
            }
            else
            {
                sprintf(Row4, "Ctrl volt:   %1.2f, V", LCDBuffer.readControlVoltage);
            }
            
            LCDhome();
            
            strcpy(Buffer, Row1);
            strcat(Buffer, Row3);
            strcat(Buffer, Row2);
            strcat(Buffer, Row4);
            Buffer[80] = '\0';
            
            LCDprint(Buffer);
            
            SetVTimerValue(LCD_REFRESH_TIMER, T_500_MS); //??????????????????????????????????????????????????????????????????/
            
            LCDAcknowledge = TRUE;
        }
        else
        {
            LCDAcknowledge = FALSE;
            LCDRequest = TRUE;
        }    
    }
}
