#include "f28x_project.h"
#include "math.h"

#define PWM_ADC_PERIOD  (501) // fs = 200 kHz
#define PWM_PERIOD      (250) // fs = 400 kHz
#define DUTY_C          (125)
#define DT_ON           (6)
#define DT_OFF          (10)
#define SAMPLE_OFFSET   (28)

#ifndef PWM_H_
#define PWM_H_

void initEPWM1(void);
void initEPWM2(void);

void ePWM_ADC_Trig(void);

void initEPWM4(void);

#pragma CODE_SECTION(DutyPWM1, ".TI.ramfunc");
#pragma CODE_SECTION(DutyPWM2, ".TI.ramfunc");

void DutyPWM1(float duty);
void DutyPWM2(float duty);


#pragma CODE_SECTION(TrigPointSel, ".TI.ramfunc");

uint16_t TrigPointSel (float duty);

#pragma CODE_SECTION(ADC_Trig_Point, ".TI.ramfunc");

void ADC_Trig_Point(float duty);

#endif
