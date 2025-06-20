/*
 * apds9960.c
 *
 *  Created on: Jun 5, 2025
 *      Author: anton
 */


#include "apds9960.h"

#define APDS9960_ENABLE      0x80
#define APDS9960_PDATA       0x9C
#define APDS9960_ID          0x92
#define APDS9960_PILT        0x89
#define APDS9960_PIHT        0x8B
#define APDS9960_PERS        0x8C
#define APDS9960_CONTROL     0x8F

uint8_t APDS9960_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t id = 0;
    HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, APDS9960_ID, 1, &id, 1, HAL_MAX_DELAY);
    if (id != 0xAB) return 0;

    uint8_t data;


    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, APDS9960_PILT, 1, &data, 1, HAL_MAX_DELAY);
    data = 0xFF;
    HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, APDS9960_PIHT, 1, &data, 1, HAL_MAX_DELAY);


    data = 0x11;
    HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, APDS9960_PERS, 1, &data, 1, HAL_MAX_DELAY);


    data = 0x02;
    HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, APDS9960_CONTROL, 1, &data, 1, HAL_MAX_DELAY);


    data = 0x05;
    HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, APDS9960_ENABLE, 1, &data, 1, HAL_MAX_DELAY);

    return 1;
}

uint8_t APDS9960_ReadProximity(I2C_HandleTypeDef *hi2c) {
    uint8_t prox = 0;
    HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, APDS9960_PDATA, 1, &prox, 1, HAL_MAX_DELAY);
    return prox;
}
