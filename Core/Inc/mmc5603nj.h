/*  Driver for the MMC5603NJ magnetic sensor.
 Copyright (C) 2023 Lana Popova.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>. */

#ifndef INC_MMC5603NJ_H_
#define INC_MMC5603NJ_H_

#include "stm32f0xx_hal.h"

typedef enum {
  MMC_WAIT,
  MMC_READY,
  MMC_ERROR
} MMC_STATUS_ENUM;

MMC_STATUS_ENUM MMC5603NJ_init(I2C_HandleTypeDef *handle_i2c);

#endif /* INC_MMC5603NJ_H_ */
