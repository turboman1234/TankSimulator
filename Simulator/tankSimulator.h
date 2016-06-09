#ifndef __TANKSIMULATOR_H
#define __TANKSIMULATOR_H

#include "definitions.h"
#include "math.h"

#define PUMP_CONTROL_VOLTAGE                    ADC_1
#define FLUID_LEVEL                             DAC_1
#define OUTPUT_FLOW                             DAC_2
#define LED_2_CM                                LED_1
#define LED_4_CM                                LED_2
#define LED_6_CM                                LED_3
#define LED_8_CM                                LED_4
#define LED_10_CM                               LED_5
#define ALARM_LED                               LED_8
#define SAMPLE_TIMER                            TIMER_1
#define LCD_REFRESH_TIMER                       TIMER_2

//Tank parameters --------------------------------------------------------------

#define GRAVITY_CONSTANT                        9.81                                                    // m/s^2
#define TANK_VOLUME                             0.001                                                   // m^3
#define H_MAX                                   0.1                                                     // m
#define S_TANK                                  TANK_VOLUME / H_MAX                                     // m^2
#define PUMP_COEF                               0.000005556                                             // m^3/s.V
#define U_MAX                                   3.0                                                     // V
#define F_IN_MAX                                PUMP_COEF * U_MAX                                       // m^3/s
#define F_OUT_MAX                               F_IN_MAX / 2.0                                          // m^3/s
#define S_OUT                                   F_OUT_MAX / sqrt(2 * GRAVITY_CONSTANT * H_MAX)          // m^2
#define SAMPLE_TIME                             T_10_MS / 10                                            // miliseconds - dicretization time

#define A                                       -(S_OUT * sqrt(2 * GRAVITY_CONSTANT)) / S_TANK          // model constant
#define B                                       PUMP_COEF / S_TANK                                      // model constant
#define T0                                      0.001                                                   // seconds 

#define TANK_LEVEL_0_CM                         0.0                                                
#define TANK_LEVEL_2_CM                         0.02
#define TANK_LEVEL_4_CM                         0.04
#define TANK_LEVEL_6_CM                         0.06
#define TANK_LEVEL_8_CM                         0.08
#define TANK_LEVEL_10_CM                        0.1

//------------------------------------------------------------------------------

#define MIN_DAC_VALUE                           96
#define MAX_DAC_VALUE                           4095
#define FLUID_LEVEL_TO_DAC_CODE                 4000.0 / (H_MAX + 0.0001)                               // H_MAX + 0.0001 ----> 0.1001 is the max fluid level which DAC could transmit
#define OUTPUT_FLOW_TO_DAC_CODE                 4000.0 / (F_OUT_MAX * 1.0005)                           // (F_OUT_MAX * 1.0005) ----> max flow by h = 0.1001, m
#define ADC_CODE_TO_VOLTAGE_CONSTANT            U_MAX / 4000.0                                          // first 95 adc codes will not be used because it is possible adc noice 


typedef struct TankStructure{
    float currentControlVoltage;
    float oldControlVoltage;
    float currentFluidLevel;
    float oldFluidLevel;
    float outputFlow;
}Tank;


void InitTankPeripheral(void);
void InitInitialCondition(void);
void ReadControlSignal(void);
void CalculateTankLevel(void);
void SetFluidLevelAndOutputFlowValuesToOutput(void);
void SetLEDLevelIndicator(void);
void TankSimulatorTask(void);

#endif
