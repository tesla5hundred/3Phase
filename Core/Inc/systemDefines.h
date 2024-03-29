/*
 * systemDefines.h
 *
 *  Created on: Feb 21, 2021
 *      Author: User
 */

#ifndef INC_SYSTEMDEFINES_H_
#define INC_SYSTEMDEFINES_H_

#define ADC_BITS	12
#define	ADC_MIN		0
#define	ADC_MAX		((1 << ADC_BITS) - 1)

#define PWM_CLK		168000000	//168MHz

#define	ISENSE_GAIN	213.48		//Gain of current sensor in units of ADC counts per amp
#define	VSENSE_FS	80.04		//Full scale range of the voltage sense divider

#define PWM_FS		(1166)	//Value that represents 100% PWM duty

#endif /* INC_SYSTEMDEFINES_H_ */
