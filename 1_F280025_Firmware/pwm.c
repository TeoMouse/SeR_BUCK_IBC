#include "pwm.h"


void initEPWM1(void){

    EALLOW;
    EPwm1Regs.TZSEL.bit.OSHT1 = 1; //Enable TZ1 as a one-shot trip source for this ePWM module
    EPwm1Regs.TZCTL.bit.TZA = 2; //Force EPWMxA to a low state
    EPwm1Regs.TZCTL.bit.TZB = 2; //Force EPWMxB to a low state
    EDIS;


    EPwm1Regs.TBCTR = 0; // Clear counter
    EPwm1Regs.TBPRD = PWM_PERIOD;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // HighSpeedTimeBaseClock Prescaler = /1
    EPwm1Regs.TBCTL.bit.CLKDIV = 0; // TimeBaseClock Prescaler = /1
    EPwm1Regs.TBPHS.bit.TBPHS = 0; // Phase is 0


    EPwm1Regs.CMPA.bit.CMPA = 0; // Set compare A value
    EPwm1Regs.CMPC= PWM_PERIOD>>1; // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = 0; // Set Compare B value

    EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Up count mode
    EPwm1Regs.TBCTL.bit.PHSEN = 0; // Enable phase loading
    //EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; // Sync output select: CTR=zero
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1; // Force 1 time sync pulse to be generated


    EPwm1Regs.AQCTLA.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm1Regs.AQCTLA.bit.ZRO = 2; // when PRD == TBCTR force output HIGH

    EPwm1Regs.AQCTLB.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm1Regs.AQCTLB.bit.ZRO = 2; // when PRD == TBCTR force output HIGH


    EPwm1Regs.DBCTL.bit.OUT_MODE = 3; // Dead band is fully enabled
    EPwm1Regs.DBCTL.bit.POLSEL = 2; // EPWMxB is inverted
    EPwm1Regs.DBCTL.bit.HALFCYCLE = 1; //Half cycle clocking is enabled
    EPwm1Regs.DBRED.bit.DBRED = DT_ON; 
    EPwm1Regs.DBFED.bit.DBFED = DT_OFF; 

    EALLOW;

    EPwm1Regs.HRCNFG.all = 0x0;
    EPwm1Regs.HRCNFG.bit.EDGMODE = 2; //MEP control on falling edge
    //EPwm1Regs.HRCNFG.bit.EDGMODEB = 1; //MEP control on rising edge
    EPwm1Regs.HRCNFG.bit.CTLMODE = 0; //CMPAHR(8) or TBPRDHR(8) Register controls the edge position
    EPwm1Regs.HRCNFG.bit.HRLOAD = 0; //Load on CTR = Zero
    EPwm1Regs.HRCNFG.bit.AUTOCONV = 1; //Automatic HRMSTEP scaling is enabled


    EPwm1Regs.HRCNFG2.bit.EDGMODEDB = 3;
    //EPwm1Regs.HRPCTL.bit.HRPE = 1;

    EDIS;

}

void initEPWM2(void){

    EALLOW;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1; //Enable TZ1 as a one-shot trip source for this ePWM module
    EPwm2Regs.TZCTL.bit.TZA = 2; //Force EPWMxA to a low state
    EPwm2Regs.TZCTL.bit.TZB = 2; //Force EPWMxB to a low state
    EDIS;


    EPwm2Regs.TBCTR = 0; // Clear counter
    EPwm2Regs.TBPRD = PWM_PERIOD;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // HighSpeedTimeBaseClock Prescaler = /1
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // TimeBaseClock Prescaler = /1
    EPwm2Regs.TBPHS.bit.TBPHS = PWM_PERIOD>>1; // Phase is 180


    EPwm2Regs.CMPA.bit.CMPA = 0; // Set compare A value
    EPwm2Regs.CMPC= PWM_PERIOD>>1; // Set compare A value
    EPwm2Regs.CMPB.bit.CMPB = 0; // Set Compare B value

    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Up count mode
    EPwm2Regs.TBCTL.bit.PHSEN = 1; // Enable phase loading
    //EPwm2Regs.TBCTL.bit.SYNCOSEL = 1; // Sync output select: CTR=zero
    EPwm2Regs.TBCTL.bit.SWFSYNC = 1; // Force 1 time sync pulse to be generated


    EPwm2Regs.AQCTLA.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // when PRD == TBCTR force output HIGH

    EPwm2Regs.AQCTLB.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // when PRD == TBCTR force output HIGH


    EPwm2Regs.DBCTL.bit.OUT_MODE = 3; // Dead band is fully enabled
    EPwm2Regs.DBCTL.bit.POLSEL = 2; // EPWMxB is inverted
    EPwm2Regs.DBCTL.bit.HALFCYCLE = 1; //Half cycle clocking is enabled
    EPwm2Regs.DBRED.bit.DBRED = DT_ON;
    EPwm2Regs.DBFED.bit.DBFED = DT_OFF;

    EALLOW;

    EPwm2Regs.HRCNFG.all = 0x0;
    EPwm2Regs.HRCNFG.bit.EDGMODE = 2; //MEP control on falling edge
    //EPwm2Regs.HRCNFG.bit.EDGMODEB = 1; //MEP control on rising edge
    EPwm2Regs.HRCNFG.bit.CTLMODE = 0; //CMPAHR(8) or TBPRDHR(8) Register controls the edge position
    EPwm2Regs.HRCNFG.bit.HRLOAD = 0; //Load on CTR = Zero
    EPwm2Regs.HRCNFG.bit.AUTOCONV = 1; //Automatic HRMSTEP scaling is enabled


    EPwm2Regs.HRCNFG2.bit.EDGMODEDB = 3;
    //EPwm2Regs.HRPCTL.bit.HRPE = 1;

    EDIS;

}

void ePWM_ADC_Trig(void){

    EPwm4Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOC on A group
    EPwm4Regs.ETSEL.bit.SOCBEN = 1;    // Enable SOC on B group

    EPwm4Regs.ETSEL.bit.SOCASEL = 4;   // Enable event time-base counter equal to CMPA when the timer is incrementing or CMPC when the timer is incrementing
    EPwm4Regs.ETSEL.bit.SOCBSEL = 6;   // Enable event: time-base counter equal to CMPB when the timer is incrementing or CMPD when the timer is incrementing

    EPwm4Regs.ETSEL.bit.SOCASELCMP = 1; // Enable event time-base counter equal to CMPC when the timer is incrementing
    EPwm4Regs.ETSEL.bit.SOCBSELCMP = 1; // Enable event time-base counter equal to CMPD when the timer is incrementing

    EPwm4Regs.ETPS.bit.SOCAPRD = 3;       // Generate pulse on 3rd event /* 200 kHz/3 = 66.67 kHz sampling */
    EPwm4Regs.ETPS.bit.SOCBPRD = 3;       // Generate pulse on 3rd event /* 200 kHz/3 = 66.67 kHz sampling */


}

void initEPWM4(void){

    EALLOW;
    EPwm4Regs.TZSEL.bit.OSHT1 = 1; //Enable TZ1 as a one-shot trip source for this ePWM module
    EPwm4Regs.TZCTL.bit.TZA = 2; //Force EPWMxA to a low state
    EPwm4Regs.TZCTL.bit.TZB = 2; //Force EPWMxB to a low state
    EDIS;

    EPwm4Regs.TBCTR = 0; // Clear counter
    EPwm4Regs.TBPRD = PWM_ADC_PERIOD;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0; // HighSpeedTimeBaseClock Prescaler = /1
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // TimeBaseClock Prescaler = /1
    EPwm4Regs.TBPHS.bit.TBPHS = 0; // Phase is 0

    EPwm4Regs.CMPA.bit.CMPA = DUTY_C; // Set compare A value
    EPwm4Regs.CMPB.bit.CMPB = 0; // Set Compare B value

    EPwm4Regs.TBCTL.bit.CTRMODE = 0; // Up count mode
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Enable phase loading

    EPwm4Regs.TBCTL.bit.SWFSYNC = 1; // Force 1 time sync pulse to be generated

    EPwm4Regs.AQCTLA.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm4Regs.AQCTLA.bit.ZRO = 2; // when PRD == TBCTR force output HIGH

    EPwm4Regs.AQCTLB.bit.CBU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm4Regs.AQCTLB.bit.ZRO = 2; // when PRD == TBCTR force output HIGH

    EPwm4Regs.DBCTL.bit.OUT_MODE = 0; // Dead band is disabled

}

#pragma CODE_SECTION(DutyPWM1, ".TI.ramfunc");

void DutyPWM1(float duty){

    Uint32 temp;
    Uint16 CMPA_reg_val, CMPAHR_reg_val;
    int duty_Q15;


    duty_Q15 = duty*32767;
    if (duty_Q15>32767) duty_Q15 = 32767;
    if (duty_Q15<-32768) duty_Q15 = -32768;

    CMPA_reg_val = ((long)duty_Q15 * (PWM_PERIOD + 1)) >> 15;
    temp = ((long)duty_Q15 * (PWM_PERIOD + 1)) ;
    temp = temp - ((long)CMPA_reg_val << 15);
    CMPAHR_reg_val = temp << 1; // convert to Q16

    EPwm1Regs.CMPA.all = ((long)CMPA_reg_val) << 16 | CMPAHR_reg_val;

    
}

#pragma CODE_SECTION(DutyPWM2, ".TI.ramfunc");

void DutyPWM2(float duty){

    Uint32 temp;
    Uint16 CMPA_reg_val, CMPAHR_reg_val;
    int duty_Q15;


    duty_Q15 = duty*32767;
    if (duty_Q15>32767) duty_Q15 = 32767;
    if (duty_Q15<-32768) duty_Q15 = -32768;

    CMPA_reg_val = ((long)duty_Q15 * (PWM_PERIOD + 1)) >> 15;
    temp = ((long)duty_Q15 * (PWM_PERIOD + 1)) ;
    temp = temp - ((long)CMPA_reg_val << 15);
    CMPAHR_reg_val = temp << 1; // convert to Q16

    EPwm2Regs.CMPA.all = ((long)CMPA_reg_val) << 16 | CMPAHR_reg_val;


}

#pragma CODE_SECTION(TrigPointSel, ".TI.ramfunc");

uint16_t TrigPointSel (float duty){

    uint16_t sel = 0;

    //sel= 1: trig @ d/2, sel = 0: trig @ d+(1-d)/2

    if (duty > 0.52f) sel = 1;
    else if (duty < 0.48f) sel = 0 ;

    return sel;
}

#pragma CODE_SECTION(ADC_Trig_Point, ".TI.ramfunc");
int Dtest1=0, Dtest2=0;

void ADC_Trig_Point(float duty){

    //sel= 1: trig @ d/2, sel = 0: trig @ d+(1-d)/2


    EPwm4Regs.CMPC = (duty + (1-duty)*0.5f) * (PWM_ADC_PERIOD>>1)+SAMPLE_OFFSET;
    EPwm4Regs.CMPD = (duty + (1-duty)*0.5f) * (PWM_ADC_PERIOD>>1) + (PWM_ADC_PERIOD>>2) +SAMPLE_OFFSET;




}
