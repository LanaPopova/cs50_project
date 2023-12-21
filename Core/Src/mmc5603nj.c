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

#include "mmc5603nj.h"

#define MMC5603NJ_ADDR 0x30U

MMC_STATUS_ENUM MMC5603NJ_init(I2C_HandleTypeDef *handle_i2c)
{
  static MMC_STATUS_ENUM mmc_status;

  switch (HAL_I2C_IsDeviceReady(handle_i2c, MMC5603NJ_ADDR, 3U, 100U))
  {
    case (HAL_BUSY):
      if ( mmc_status != MMC_WAIT )
      {
        mmc_status = MMC_WAIT;
      }
      break;
    case (HAL_OK):
      // TODO: add chip initialization
      mmc_status = MMC_READY;
      break;
    case (HAL_TIMEOUT):
    case (HAL_ERROR):
    default:
      mmc_status = MMC_ERROR;
      break;
  }

  return mmc_status;
}
