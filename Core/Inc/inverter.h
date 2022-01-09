/*
 * inverter.h
 *
 *  Created on: Dec 19, 2021
 *      Author: User
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

#include <stdint.h>

EXTERNC void inverterInit(void);
EXTERNC void loop(uint16_t vMeas, uint16_t iMeas, uint16_t vMeas2, uint16_t iMeas2, uint16_t forceIRef);


#endif /* INC_INVERTER_H_ */
