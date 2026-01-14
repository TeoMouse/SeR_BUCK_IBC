
#include "f28x_project.h"
#include "math.h"
#include "sfo_v8.h"
#include "adc.h"
#include "pwm.h"
#include "control.h"

#define VIN_CONV_ON     (46)    // Allow converter to operate if Vin > VIN_CONV_ON
#define VIN_CONV_OFF    (42)    // Disable converter if Vin < VIN_CONV_OFF
#define VI_OVP          (50)    // Input voltage for operation after the OVP is triggered
#define VI_OVP_OFF      (52)    // Maximum input voltage for normal operation
#define MAX_TEMP_TH_OFF (105)   // Maximum allowable operating Temp
#define MAX_TEMP_TH_ON  (80)    // Temp on which the temperature fault resets
#define DUTY_MAX        (0.38f) // Maximum allowable duty cycle

#define V_REF           (6.f)   // Output voltage reference

#define VO_OVP          (7.f)   // Maximum Vout for OVP
#define VO_OVP_REP      (3)     // # of repetitions + 1 for Vout overvoltage to trigger 

#define VO_UVL          (5.f)   // Minimum Vout for UVLO at steady state
#define VO_UVL_REP      (9)     // # of repetitions + 1 for Vout undervoltage to trigger (Steady state)

#define IPH_OCP_H       (40)    // Maximum current per phase for OCP (High OCP limit)
#define IPH_OCP_L       (30)    // Maximum current per phase for OCP (Low OCP limit)
#define IPH_OCP_L_REP   (1000)  // # of repetitions + 1 for OCP Low Limit to trigger (~ 15 ms)
#define IPH_OCP_H_REP   (1)     // # of repetitions + 1 for OCP High Limit to trigger

#define OVP_MAX_FAULTS  (5)     // Maximum number of retries if OVP fault occurs
#define OVP_RETRY_CNT   (20)    // After OVP fault retry every OVP_Cnt_R ms

#define UVP_MAX_FAULTS  (8)     // Maximum number of retries if UVP fault occurs
#define UVP_RETRY_CNT   (20)    // After OVP fault retry every UVP_Cnt_R ms

#define D_STEP_RAMP     (0.001f)// Duty cycle step during start up
#define SS_UP_CNT_VALUE (3)     // Duty cycle during start up increases every 3 cycles

#define TIMER0_CNT      (1000)  //timer 0 1000us count
#define TIMER1_CNT      (20)    //timer 1 20us count
#define MEP_CAL_CNT     (20000) //Calibrate MEP every 20000 cycles x 1 ms
#define LED_CNT_C       (500)   //LED counter resets every 500 x 1 ms

#define PWM_CH            3       // # of PWM channels + 1
volatile struct EPWM_REGS *ePWM[PWM_CH] = {0, &EPwm1Regs, &EPwm2Regs};
int duty_Q15;

int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO() function.
void error(void);

uint16_t status_HR;

/* Load and execute interrupt function from RAM */
#pragma CODE_SECTION(adcA1ISR, ".TI.ramfunc");
#pragma CODE_SECTION(adcC1ISR, ".TI.ramfunc");

__interrupt void adcA1ISR(void);
__interrupt void adcC1ISR(void);

/* -- Lag Controller + Integrator voltage controller definitions -- */
float ConA[3] = {1, -1.7802, 0.7802}, ConB[3] = {0.0028, 0.00009, -0.0018};
float ConOut_Vout[3] = {0,0,0}, Err_Vout[3]={0,0,0}; 

/* -- LOW PASS FILTER with Fc @ ~0.13 kHz definitions (Fs = 1kHz) -- */

float af[3] = {1, -0.4493, 0}, bf[3] = {0, 0.5507, 0};
float Temp_x[3] = {0, 0, 0}, Temp_y[3] = {0, 0 ,0};
float Iacs_x[3] = {0, 0, 0}, Iacs_y[3] = {0, 0 ,0};


void main(void){
    //
    // Initialize device clock and peripherals
    //
     InitSysCtrl();

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    InitGpio();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts. 
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt routines
    InitPieVectTable();

    // Enable PIE interrupt
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

    EALLOW;
    PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt
    PieVectTable.ADCC1_INT = &adcC1ISR;     // Function for ADCC interrupt
    EDIS;


    // Enable global Interrupts and higher priority real-time debug events:
    IER |= M_INT1;  // Enable group 1 interrupts


    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    ConfigureADC();
    ePWM_ADC_Trig();
    SetupADC();
    //InitLEDGPIOs();


    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 100, TIMER0_CNT); // Configure counter to count TIMER0_CNT us
    CpuTimer0Regs.TCR.all = 0x4000; //Clear timer0 flag

    ConfigCpuTimer(&CpuTimer1, 100, TIMER1_CNT); // Configure counter to count TIMER1_CNT us
    CpuTimer1Regs.TCR.all = 0x4000; //Clear timer0 flag

    while(status_HR == SFO_INCOMPLETE){

        status_HR = SFO();
        if(status_HR == SFO_ERROR){

             error();   // SFO function returns 2 if an error occurs & # of MEP
         }              // steps/coarse step exceeds maximum of 255.
    }

    DELAY_US(10000); // Delay 10 ms

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    //InitEPwm4Gpio(); //Only for debugging
    initEPWM1();

    initEPWM2();

    initEPWM4();

    ConverterOFF();


    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    while(1);
}

float Vin_Meas, Vout_Meas, Iind1_Meas, Iind2_Meas;
int16_t Out_CNT;
int16_t OCP_Cnt_Iind_H1, OCP_Cnt_Iind_L1, OCP_Cnt_Iind_H2, OCP_Cnt_Iind_L2;
int16_t UVL_Cnt, OVP_Cnt;
int16_t Latch_Fault, Vprot_UVP, UVP_Faults, I_prot, Vprot_OVP, OVP_Faults;

int16_t Conv_EN = 0, Conv_SS = 0, StartUP_cnt, select, CL_Latch = 1;

float D_Thresh_SS, D_total, Duty_OL = 0.25f,Vref = 6;


__interrupt void adcA1ISR(void){

    //LED_Red_Toggle();

    Iind1_Meas = GetIind1(); //Get inductor voltage measurement
    Iind2_Meas = GetIind2(); //Get inductor voltage measurement
    Vout_Meas = GetVout(); //Get the Vout voltage measurement
    Vin_Meas = GetVin(); //Get the Vin voltage measurement

    /* if Iind > threshold for IPH_OCP_H_REP+1(high limit) or IPH_OCP_L_REP+1(low limit) 
     * consecutive measurements then converter off */
    if(Iind1_Meas > IPH_OCP_H) OCP_Cnt_Iind_H1++;
    else{
        OCP_Cnt_Iind_H1--;
        if(OCP_Cnt_Iind_H1<0) OCP_Cnt_Iind_H1 = 0;
    }

    if(Iind1_Meas > IPH_OCP_L) OCP_Cnt_Iind_L1++;
    else{
        OCP_Cnt_Iind_L1--;
        if(OCP_Cnt_Iind_L1<0) OCP_Cnt_Iind_L1 = 0;
    }

    if(Iind2_Meas > IPH_OCP_H) OCP_Cnt_Iind_H2++;
    else{
        OCP_Cnt_Iind_H2--;
        if(OCP_Cnt_Iind_H2<0) OCP_Cnt_Iind_H2 = 0;
    }

    if(Iind2_Meas > IPH_OCP_L) OCP_Cnt_Iind_L2++;
    else{
        OCP_Cnt_Iind_L2--;
        if(OCP_Cnt_Iind_L2<0) OCP_Cnt_Iind_L2 = 0;
    }

    if (OCP_Cnt_Iind_H1 > IPH_OCP_H_REP || OCP_Cnt_Iind_H2 > IPH_OCP_H_REP || OCP_Cnt_Iind_L1 > IPH_OCP_L_REP || OCP_Cnt_Iind_L2 > IPH_OCP_L_REP ){
        ConverterOFF();
        Latch_Fault = 1;
        I_prot = 1;
        Conv_EN = 0;
    }

    /* if Vout < threshold for VO_UVL_REP+1 consecutive measurements during steady
     * state operation then converter off */
    if ((Vout_Meas < VO_UVL) && (Conv_SS == 1)) UVL_Cnt++;
    else if ((Vout_Meas > VO_UVL) && (Conv_SS == 1)) {
        UVL_Cnt--;
        if(UVL_Cnt<0) UVL_Cnt = 0;
    }

    if (UVL_Cnt > VO_UVL_REP) {
        ConverterOFF();
        Latch_Fault = 1;
        Vprot_UVP = 1;
        UVP_Faults++;
        Conv_EN = 0;
    }

    /* if Vout > threshold for VO_OVP_REP+1 consecutive measurements then converter off */
    if (Vout_Meas > VO_OVP) OVP_Cnt++;
    else {
        OVP_Cnt--;
        if(OVP_Cnt<0) OVP_Cnt = 0;
    }

    if (OVP_Cnt > VO_OVP_REP) {
        ConverterOFF();
        Latch_Fault = 1;
        Vprot_OVP = 1;
        OVP_Faults++;
        Conv_EN = 0;
    }




    if (Conv_EN == 1 && Latch_Fault == 0){
        
        /* If Converter Enable  == 1 and there is no fault*/
        switch(Conv_SS){
            case 0: //Converter is in start up phase
                StartUP_cnt++;
                D_Thresh_SS = 2.2f*V_REF/Vin_Meas;
                D_Thresh_SS = saturator(0.1f, DUTY_MAX, D_Thresh_SS);

                if(StartUP_cnt>SS_UP_CNT_VALUE){

                    StartUP_cnt = 0;

                    D_total = D_total + D_STEP_RAMP;

                    // Update duty cycle

                    DutyPWM1(D_total);
                    DutyPWM2(D_total);

                    /* Assign trigger point for ADC channels */
                    ADC_Trig_Point(D_total);

                }
                /* if duty cycle ramp during start up reaches D_THRESH_SS,
                 * then switch to steady state operation closed loop operation */
                if (D_total> D_Thresh_SS || (Vout_Meas> 0.99f*V_REF) ){

                    StartUP_cnt = 0;
                    Conv_SS = 1;    //Start up is over and Steady State operation is established

                    /* Assign initial values at the controllers to eliminate
                     * harsh transients during the transition */

                    ConOut_Vout[0] = D_total;
                    ConOut_Vout[1] = D_total;
                    ConOut_Vout[2] = D_total;
                    D_total = 0;

                }
                break;

            case 1: // Converter is in steady state operation

                switch(CL_Latch){
                    case 1: //Closed loop operation 

                         // Lag controller + Integrator
                        Err_Vout[0] = (Vref - Vout_Meas);
                        /*if (Err_Vout[0]>0.15f && Err_Vout[0]>0){
                            Err_Vout[0] = 3.f*Err_Vout[0];
                        }
                        else if(Err_Vout[0]<-0.15f && Err_Vout[0]<0){
                            Err_Vout[0] = 3.f*Err_Vout[0];
                        }*/

                        ZPIController(ConOut_Vout,Err_Vout, ConA, ConB);
                        ConOut_Vout[0] = saturator(0, DUTY_MAX, ConOut_Vout[0]);

                        /* Update duty cycle*/
                        DutyPWM1(ConOut_Vout[0]);
                        DutyPWM2(ConOut_Vout[0]);

                        /* Assign trigger point for ADC channels */
                        ADC_Trig_Point(ConOut_Vout[0]);

                        /* Shift array values for the controllers */

                        shift(ConOut_Vout,3);
                        shift(Err_Vout,3);

                        break;

                    case 0: // Open loop operation

                        /* Update duty */
                        Duty_OL = saturator(0.1f, DUTY_MAX, Duty_OL);
                        DutyPWM1(Duty_OL);
                        DutyPWM2(Duty_OL);

                        /* Assign trigger point for ADC channels */
                        ADC_Trig_Point(Duty_OL);

                        break;

                    }

                    break;

                }
    }
    else{
        /* If Converter Enable  == 0 OR there is a fault*/

        /* Update duty cycle and set it equal to zero*/
        DutyPWM1(0.f);
        DutyPWM2(0.f);

        /* Clear arrays and initialize values for start up */
        StartUP_cnt = 0;
        Conv_SS = 0;
        CLR_Array(ConOut_Vout,3);
        CLR_Array(Err_Vout,3);
        D_total = 0;
        OVP_Cnt = 0;
        UVL_Cnt = 0;
        OCP_Cnt_Iind_L1 = 0;
        OCP_Cnt_Iind_H1 = 0;
        OCP_Cnt_Iind_L2 = 0;
        OCP_Cnt_Iind_H2 = 0;

    }

    // Acknowledge the interrupt
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

int16_t Vin_UVP, Vin_OVP, TempProt, CLR_Fault, OVP_Cnt_R, OVP_Faults, HardLatch_OVP, UVP_Cnt_R, UVP_Faults, HardLatch_UVP;
float Temp_MeasF=0;

int16_t ConvCTRL, ConvCTRL_cnt1, ConvCTRL_cnt2, ConvStatus;

int16_t MEP_Cnt;
 
__interrupt void adcC1ISR(void){

    /* Check for minimum and maximum operating voltage limits */
    if ((Vin_Meas > VIN_CONV_ON) && (Vin_Meas < VI_OVP) && (Latch_Fault == 0)) {
        
        Conv_EN  = 1;
        Vin_OVP = 0;
        Vin_UVP = 0;

        if (Conv_SS == 0){
            /* Clear tripzone flag for each phase */
            EALLOW;
            EPwm1Regs.TZCLR.bit.OST = 1; //CLR TZ status converter
            EPwm2Regs.TZCLR.bit.OST = 1; //CLR TZ status converter
            
            /* Trigger ADC conversions according to PWM */
            AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //SOC0 conversion trigger -> ePWM4
            AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 12; //SOC1 conversion trigger -> ePWM4
            AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 12; //SOC2 conversion trigger -> ePWM4
            AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 12; //SOC3 conversion trigger -> ePWM4
            EDIS;
        }
    }

    if (Vin_Meas < VIN_CONV_OFF){
        Conv_EN =  0;
        ConverterOFF();
        Vin_UVP = 1;
    }

    if (Vin_Meas > VI_OVP_OFF){
        Conv_EN =  0;
        ConverterOFF();
        Vin_OVP = 1;
    }


    /* Get temperature value */
    Temp_x[0] = GetTemp();
    secfilt(Temp_x , Temp_y, af, bf, 1);
    Temp_MeasF = Temp_y[0];

    /* Shift the arrays for Temp filters */
    shift(Temp_x,3);
    shift(Temp_y,3);

    /* Check for over temperature protection*/
    if (Temp_MeasF > MAX_TEMP_TH_OFF){
        ConverterOFF();
        Latch_Fault = 1;
        TempProt = 1;
    }
    if (TempProt == 1 && Temp_MeasF < MAX_TEMP_TH_ON) CLR_Fault = 1;

    if (Latch_Fault == 1 && Vprot_OVP == 1){
        OVP_Cnt_R ++;

        if (OVP_Faults < OVP_MAX_FAULTS && OVP_Cnt_R > OVP_RETRY_CNT && HardLatch_OVP == 0){
            CLR_Fault = 1;
        }
        else if (OVP_Faults >= OVP_MAX_FAULTS){
            OVP_Faults = 0;
            OVP_Cnt_R = 0;
            HardLatch_OVP = 1;
        }
    }

    if (Latch_Fault == 1 && Vprot_UVP == 1){
        UVP_Cnt_R ++;
        if (UVP_Faults < UVP_MAX_FAULTS && UVP_Cnt_R > UVP_RETRY_CNT && HardLatch_UVP == 0){
            CLR_Fault = 1;
        }
        else if (UVP_Faults >= UVP_MAX_FAULTS){
            UVP_Faults = 0;
            UVP_Cnt_R = 0;
            HardLatch_UVP = 1;
        }
    }

    // Clear fault and remove the latch condition
    if (CLR_Fault == 1 || Vin_Meas < VIN_CONV_OFF){
        Latch_Fault = 0;
        CLR_Fault = 0;
        OVP_Cnt_R = 0;
        UVP_Cnt_R = 0;
        HardLatch_OVP = 0;
        HardLatch_UVP = 0;

        Vprot_UVP = 0;
        Vprot_OVP = 0;
        I_prot = 0;
        TempProt = 0;

    }


    //Run Calibration module for MEP every MEP_CAL_CNT cycles x 1ms
    MEP_Cnt++;
    if (MEP_Cnt> MEP_CAL_CNT){

        MEP_Cnt = 0;

        // Call the scale factor optimizer lib function SFO()
        // periodically to track for any change due to temp/voltage.
        // This function generates MEP_ScaleFactor by running the
        // MEP calibration module in the HRPWM logic. This scale
        // factor can be used for all HRPWM channels. The SFO()
        // function also updates the HRMSTEP register with the
        // scale factor value.


        status_HR = SFO(); // in background, MEP calibration module
                           // continuously updates MEP_ScaleFactor

        if(status_HR == SFO_ERROR){

         error();   // SFO function returns 2 if an error occurs & #
                    // of MEP steps/coarse step
        }

    }

    // Acknowledge the interrupt
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void error (void){

    ESTOP0;         // Stop here and handle error
}

//
// End of File
//
