/*
 * inverter.cpp
 *
 *  Created on: Dec 19, 2021
 *      Author: User
 */

#include "inverter.h"
#include "math.h"
#include "FastPID.h"
#include "systemDefines.h"
#include "stm32f4xx_hal.h"
#include "fpsin.h"

float Kp=1, Ki=0.016*18000, Kd=0.0, Hz=18000;
int output_bits = 16;
bool output_signed = true;

FastPID pid_i(Kp, Ki, Kd, Hz, output_bits, output_signed);
FastPID pid_i_2(Kp, Ki, Kd, Hz, output_bits, output_signed);

float Kp_v=0.16*4, Ki_v=0.02*18000, Kd_v=0.0;

FastPID pid_v(Kp_v, Ki_v, Kd_v, Hz, output_bits, output_signed);
FastPID pid_v_2(Kp_v, Ki_v, Kd_v, Hz, output_bits, output_signed);

void inverterInit(void)
{
	pid_i.setOutputRange(40, PWM_FS - 40);

	if (pid_i.err())
	{

	  for (;;) {};
	}

	pid_v.setOutputRange(0, ADC_MAX);

	if (pid_v.err())
	{

	  for (;;) {};
	}

	pid_i_2.setOutputRange(40, PWM_FS - 40);	//The lower the first value, the closer the output can get to full. This effects the bootstrap cap charging

	if (pid_i.err())
	{

	  for (;;) {};
	}

	pid_v_2.setOutputRange(0, ADC_MAX);

	if (pid_v.err())
	{

	  for (;;) {};
	}
}

#define SQUARE_WAVE(theta)	(((theta) & 0b0100000000000000) ? 1 : -1)

void loop(uint16_t vMeas, uint16_t iMeas, uint16_t vMeas2, uint16_t iMeas2, uint16_t forceIRef)
{
	//static float theta = 0;
	static int16_t iTheta = 0;
	const int16_t thetaIncrement = (int16_t)(32768.0*60.0*2.0*(float)PWM_FS/(float)PWM_CLK);

	int16_t iref =  (uint16_t)pid_v.step(fpsin(iTheta)*29*50/4096 + 35*50, vMeas);	//Setpoint scaling: 0 = 0V,  4095 = 83V; 50 = 1V
	int16_t iref2 =  (uint16_t)pid_v_2.step(fpsin(iTheta+16384)*29*50/4096 + 35*50, vMeas2);	//Setpoint scaling: 0 = 0V,  4095 = 83V; 50 = 1V
//	int16_t iref = 2140 + 14 * SQUARE_WAVE(iTheta);//forceIRef;
//	int16_t iref =  (uint16_t)pid_v.step((uint16_t)((8.0*sin(theta)+24.0)*50.0), vMeas);	//Setpoint scaling: 0 = 0V,  4095 = 83V; 50 = 1V

	TIM1->CCR1 = (uint16_t)pid_i.step(ADC_MAX - iref, iMeas);	//Fix polarity of Iref
	TIM1->CCR2 = (uint16_t)pid_i_2.step(ADC_MAX - iref2, iMeas2);	//Fix polarity of Iref
/*
	theta += 2*M_PI*60.0*2.0*(float)PWM_FS/(float)PWM_CLK;
	if(theta >= 2*M_PI)
		theta -= 2*M_PI;
*/
	iTheta += thetaIncrement;


}

