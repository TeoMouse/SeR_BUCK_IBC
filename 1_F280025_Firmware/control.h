#include "f28x_project.h"
#include "math.h"

#ifndef CONTROL_H_
#define CONTROL_H_

/* implementation of digitalPI controller */
#pragma CODE_SECTION(PIcontroller, ".TI.ramfunc");

void PIcontroller(float *OutputArray, float *ErrorArray, float kp, float ki, float Ts);

#pragma CODE_SECTION(ZPIController, ".TI.ramfunc");

void ZPIController(float *OutputArray, float *ErrorArray, float *CoefA, float *CoefB);

/* saturate input value at min and maximum values*/
#pragma CODE_SECTION(saturator, ".TI.ramfunc");

float saturator(float min, float max, float value);

#pragma CODE_SECTION(ConverterOFF, ".TI.ramfunc");

void ConverterOFF(void);

void InitLEDGPIOs(void);

void LED_GREEN_ON(void);

void LED_GREEN_OFF(void);

void LED_RED_ON(void);

void LED_RED_OFF(void);

void LED_Red_Toggle(void);

void LED_Green_Toggle(void);


#endif

