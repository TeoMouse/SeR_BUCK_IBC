#include "control.h"

/* Execute PI controller function from RAM */
#pragma CODE_SECTION(PIcontroller, ".TI.ramfunc");

void PIcontroller(float *OutputArray, float *ErrorArray, float kp, float ki, float Ts){

    OutputArray[0] = (kp+ki*Ts)*ErrorArray[0] -kp*ErrorArray[1] + OutputArray[1];
}

#pragma CODE_SECTION(ZPIController, ".TI.ramfunc");

void ZPIController(float *OutputArray, float *ErrorArray, float *CoefA, float *CoefB){

    OutputArray[0] = CoefB[0] * ErrorArray[0];
    OutputArray[0] += CoefB[1] * ErrorArray[1];
    OutputArray[0] += CoefB[2] * ErrorArray[2];
    OutputArray[0] -= CoefA[1] * OutputArray[1];
    OutputArray[0] -= CoefA[2] * OutputArray[2];
}

/* Execute saturation function from RAM */
#pragma CODE_SECTION(saturator, ".TI.ramfunc");

float saturator(float min, float max, float value){

    if (value >= max) value = max;
    if (value <= min) value = min;

    return value;
}


#pragma CODE_SECTION(ConverterOFF, ".TI.ramfunc");

void ConverterOFF(void){

    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1; //Force TZ generation for Conv. PWM
    EPwm2Regs.TZFRC.bit.OST = 1; //Force TZ generation for Conv. PWM
    //EPwm4Regs.TZFRC.bit.OST = 1; //Force TZ generation for ADC trig. PWM


    /* Trigger Iind, Iacs, Vin and Vout measurement from timer 1 */
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 2;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 2;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 2;


    EDIS;
}

void InitLEDGPIOs(void){
    EALLOW;

    // GPIO29 is LED1
    GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0; // Pin MUXING: Configure GPIO29 as GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1; // Disable Pull-Up
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1; //GPIO29 is output

    // GPIO5 is LED2
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0; // Pin MUXING: Configure GPIO13 as GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1; // Disable Pull-Up
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1; //GPIO5 is output

    EDIS;

}

void LED_GREEN_ON(void){
    GpioDataRegs.GPASET.bit.GPIO29 = 1;
}

void LED_GREEN_OFF(void){
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;
}

void LED_RED_ON(void){
    GpioDataRegs.GPASET.bit.GPIO5 = 1;
}

void LED_RED_OFF(void){
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
}

void LED_Red_Toggle(void){

    if (GpioDataRegs.GPADAT.bit.GPIO5 == 1) LED_RED_OFF();
    else LED_RED_ON();


}

void LED_Green_Toggle(void){

    if (GpioDataRegs.GPADAT.bit.GPIO29 == 1) LED_GREEN_OFF();
    else LED_GREEN_ON();

}


