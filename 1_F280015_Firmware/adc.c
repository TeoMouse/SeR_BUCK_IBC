#include "adc.h"

void ConfigureADC(void){

    // SetVREF - Set Vref mode. Function to select reference mode and offset trim.
    // Offset trim for Internal VREF 3.3 is unique.  All other modes use the same
    // offset trim. Also note that when the mode parameter is ADC_EXTERNAL, the
    // ref parameter has no effect.
    //
    // In this device, the bandgaps are common for all the ADC instances,
    // hence common Vref configuration needs to be done for all the ADCs. This
    // API configures same Vref configuration for all the supported ADCs in the
    // device.

    // Configure the reference mode for all ADCs (internal or external).
    SetVREF(ADC_ADCC, ADC_EXTERNAL, ADC_VREF3P3); //3.3V reference VrefHi tied to VDDA and external reference for 0 offset
    EALLOW;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 0; //set ADCCLK divider to /1 (for ADC A)

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; //interrupt pulse generation occurs at the end of the conversion
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; //power up ADCA
    DELAY_US(1000); //delay for 1ms to allow ADC time to power up

    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK  divider to /4 (for ADC C)

    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1; //interrupt pulse generation occurs at the end of the conversion
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; //power up ADCC
    EDIS;

    DELAY_US(1000); //delay for 1ms to allow ADC time to power up


}

void SetupADC(void){

    EALLOW;

    //A2 analog input enabled on the GPIO224
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO224 = 1; // Analog functionality enabled
    GpioCtrlRegs.GPHAMSEL.bit.GPIO224 = 1; //Analog Mode select (A2 Channel)

    //A3 analog input enabled on the GPIO242
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO242 = 1; // Analog functionality enabled
    GpioCtrlRegs.GPHAMSEL.bit.GPIO242 = 1; //Analog Mode select (A3 Channel)

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0  /*Iind1_Meas*/
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 12; //SOC0 conversion trigger -> ePWM4 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2;  //SOC1 will convert pin A2 /*Iind2_Meas*/
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11; //SOC1 conversion trigger -> ePWM4 SOCB

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 3;  //SOC2 will convert pin A3 /*Vout_Meas*/
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 11; //SOC2 conversion trigger -> ePWM4 SOCB

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 5;  //SOC3 will convert pin A5 /*Vin_Meas*/
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 11; //SOC3 conversion trigger -> ePWM4 SOCB

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //EOC3 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADC_C Measurements

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 11;  //SOC0 will convert pin C11 /*Module Temp*/
    AdccRegs.ADCSOC0CTL.bit.ACQPS = SAMPLE_WIN2; //sample window is SAMPLE_WIN2+1 sysclocks
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 1; //SOC0 conversion trigger -> Timer0

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //EOC0 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared


    EDIS;
}


/* Execute the functions to get ADC values from RAM */
#pragma CODE_SECTION(GetVin, ".TI.ramfunc");
#pragma CODE_SECTION(GetVout, ".TI.ramfunc");
#pragma CODE_SECTION(GetIind1, ".TI.ramfunc");
#pragma CODE_SECTION(GetIind2, ".TI.ramfunc");

float GetIind1(void){
    float Val = 0;

    Val = 0.01612f * AdcaResultRegs.ADCRESULT0 - 10.24f; /*Range: [10.24 A, 55.76 A]*/

    return Val;
}

float GetIind2(void){
    float Val = 0;

    Val = 0.01612f * AdcaResultRegs.ADCRESULT1 - 10.24f; /*Range: [10.24 A, 55.76 A]*/

    return Val;
}


float GetVout(void){
    float Val = 0;

    Val = 0.00347f * AdcaResultRegs.ADCRESULT2; /*Range: [0 V, 14.19 V]*/

    return Val;
}

float GetVin(void){
    float Val = 0;

    Val = 0.01802f * AdcaResultRegs.ADCRESULT3; /*Range: [0 V, 73.8 V]*/

    return Val;
}



#pragma CODE_SECTION(GetTemp, ".TI.ramfunc");

float GetTemp(void){

    float Val = 0;
    uint16_t ADC_Res = AdccResultRegs.ADCRESULT0;

    if (ADC_Res > THRS2){
        Val =  TEMP_C31*ADC_Res+ TEMP_C30;
    }
    else if (ADC_Res > THRS1){
        Val = TEMP_C22*ADC_Res*ADC_Res + TEMP_C21*ADC_Res+ TEMP_C20;
    }
    else{
        Val = TEMP_C12*ADC_Res*ADC_Res + TEMP_C11*ADC_Res+ TEMP_C10;
    }

    return Val;
}




/* Execute the functions to manipulate arrays from RAM */
#pragma CODE_SECTION(shift, ".TI.ramfunc");
#pragma CODE_SECTION(CLR_Array, ".TI.ramfunc");
#pragma CODE_SECTION(secfilt, ".TI.ramfunc");

void shift(float *array, int length){

    int i;
    for (i = length;i > 1;i--) array[i-1] = array[i-2];

    return;
}


void CLR_Array(float *array, int length){

    int i;
    for (i = 0;i < length;i++) array[i] = 0;
}

/* 2nd Order Filter Section */
void secfilt(float *in, float *y, float *a,float *b, float g){

    y[0] = b[0] * in[0]*g;
    y[0] += b[1] * in[1]*g;
    y[0] += b[2] * in[2]*g;
    y[0] -= a[1] * y[1];
    y[0] -= a[2] * y[2];

}
