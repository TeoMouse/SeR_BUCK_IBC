#include "f28x_project.h"
#include "math.h"

#define SAMPLE_WIN      (10)
#define SAMPLE_WIN2     (100)


/* 2nd order appr. for ADC value in the range [49, 0]*/
#define TEMP_C12        (0.0227f)
#define TEMP_C11        (-2.7052f)
#define TEMP_C10        (170.15f)

/* 2nd order appr. for ADC value in the range [350, 50]*/
#define TEMP_C22        (0.0007f)
#define TEMP_C21        (-0.4864f)
#define TEMP_C20        (109.01f)

/* Linear appr. for ADC value in the range [351, 1700]*/
#define TEMP_C31        (-0.0459f)
#define TEMP_C30        (35.151f)

#define THRS1			(49)  //1st threshold to change the function for temp reading
#define THRS2			(350) //2nd threshold to change the function for temp reading

#ifndef ADC_H_
#define ADC_H_


void ConfigureADC(void);

void SetupADC(void);

/* Execute the functions to get ADC values from RAM */
#pragma CODE_SECTION(GetVin, ".TI.ramfunc");
#pragma CODE_SECTION(GetVout, ".TI.ramfunc");
#pragma CODE_SECTION(GetIind1, ".TI.ramfunc");
#pragma CODE_SECTION(GetIind2, ".TI.ramfunc");

float GetIind1(void);
float GetIind2(void);
float GetVout(void);

float GetVin(void);




#pragma CODE_SECTION(GetTemp, ".TI.ramfunc");

float GetTemp(void);

/* Execute the functions to manipulate arrays from RAM */
#pragma CODE_SECTION(shift, ".TI.ramfunc");
#pragma CODE_SECTION(CLR_Array, ".TI.ramfunc");
#pragma CODE_SECTION(secfilt, ".TI.ramfunc");

void shift(float *array, int length);


void CLR_Array(float *array, int length);

/* 2nd Order Filter Section */
void secfilt(float *in, float *y, float *a,float *b, float g);

#endif

