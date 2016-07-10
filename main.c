#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include "definitions.h"
#include "rcc.h"
#include "initPeripheral.h"
#include "userLibrary.h"
#include "VTimer.h"
#include "usart.h"
#include "mytim.h"
#include "serial.h"
#include "mbslave.h"
#include "rs232.h"
#include "LCD.h"
#include "tankSimulator.h"

////
//////---------------------------
//#include "stdio.h"
//#define SIM_OUT_1               DAC_1
//#define SIM_IN_1                ADC_2
//
//#define SIM_OUT_2               DAC_2
//#define SIM_IN_2                ADC_1
//
//int output1Val = 0, output2Val = 0, trimmer;
//int adc2 = 0, adc1 = 0;
//
////---------------------------
//
//int main()
//{
//    char buff[50];
//    
//    InitADC(SIM_IN_1);
//    InitADC(SIM_IN_2);
//    
//    InitDAC(DAC_1, 0);
//    InitDAC(DAC_2, 0);
//    
//    InitButton(BUTTON_1);
//    InitButton(BUTTON_2);
//    InitButton(BUTTON_3);
//    InitButton(BUTTON_4);
//    
//    InitTrimmer(TRIMMER_1);
//    
////    InitVTimers();
////    
////    InitLCD();
////    
////    SetVTimerValue(TIMER_1, T_1_S);
////    
//    while(1)
//    {
//        if(GetButtonState(BUTTON_1) == PRESSED)
//        {
//            output1Val = 2048;
//        }
//        if(GetButtonState(BUTTON_2) == PRESSED)
//        {
//            output1Val = 0;
//        }
//        
//        if(GetButtonState(BUTTON_3) == PRESSED)
//        {
//            output2Val = 2048;
//        }
//        if(GetButtonState(BUTTON_4) == PRESSED)
//        {
//            output2Val = 0;
//        }
//        
//        trimmer = GetTrimmerValue(TRIMMER_1) * 4;
//        
//        SetAnalogOutput(SIM_OUT_1, trimmer);
//        SetAnalogOutput(SIM_OUT_2, trimmer);
//        
////        SetAnalogOutput(SIM_OUT_1, output1Val);
////        SetAnalogOutput(SIM_OUT_2, output2Val);
//        
//        adc2 = GetAnalogInput(SIM_IN_1);
//        adc1 = GetAnalogInput(SIM_IN_2);
//        
////        sprintf(buff, "ADC1: %4dADC2: %4dDAC1: %4dDAC2: %4d", adc1, adc2, output1Val, output2Val);
////        
////        if(IsVTimerElapsed(TIMER_1))
////        {
////            LCDhome();
////            
////            LCDprint(buff);
////            
////            SetVTimerValue(TIMER_1, T_1_S);
////        }
//    }
//    return 0;
//}
    
int main()
{
    //     Init peripheral
    InitRCC();
    InitVTimers();
    InitTankPeripheral();
    
    InitInitialCondition();
    InitLCD();
    
    while(1)
    {
        //do whatever you do here
        //TankSimulatorTask();
        SimulatorDisplayDataTask();
    }
    
    return 0;
}
