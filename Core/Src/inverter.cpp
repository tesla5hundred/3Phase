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


float Kp=5, Ki=0.5*18000, Kd=0.0, Hz=18000;
int output_bits = 16;
bool output_signed = true;

FastPID pid_i(Kp, Ki, Kd, Hz, output_bits, output_signed);

float Kp_v=0.08, Ki_v=0.02*18000, Kd_v=0.0;

FastPID pid_v(Kp_v, Ki_v, Kd_v, Hz, output_bits, output_signed);

void inverterInit(void)
{
	pid_i.setOutputRange(100, PWM_FS - 100);

	if (pid_i.err())
	{

	  for (;;) {};
	}

	pid_v.setOutputRange(0, ADC_MAX);

	if (pid_v.err())
	{

	  for (;;) {};
	}
}

void loop(uint16_t vMeas, uint16_t iMeas)
{
	static float theta = 0;

	int16_t iref =  (uint16_t)pid_v.step((uint16_t)((8.0*sin(theta)+24.0)*50.0), vMeas);	//Setpoint scaling: 0 = 0V,  4095 = 83V; 50 = 1V

	TIM1->CCR1 = (uint16_t)pid_i.step(ADC_MAX - iref, iMeas);	//Fix polarity of Iref

	theta += 2*M_PI*60.0*2.0*(float)PWM_FS/(float)PWM_CLK;
	if(theta >= 2*M_PI)
		theta -= 2*M_PI;

}

