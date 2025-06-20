/*
 * apds9960.h
 *
 *  Created on: Jun 5, 2025
 *      Author: anton
 */

#ifndef INC_APDS9960_H_
#define INC_APDS9960_H_

#include "stm32f4xx_hal.h"

#define APDS9960_I2C_ADDR (0x39 << 1)

uint8_t APDS9960_Init(I2C_HandleTypeDef *hi2c);
uint8_t APDS9960_ReadProximity(I2C_HandleTypeDef *hi2c);

#endif /* INC_APDS9960_H_ */
