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
#include <stdio.h>

#define MMC5603NJ_ADDR 0x60U // Device I2C address shifted to the left by 1 bit.

static uint8_t buffer_mmc[32] = {0};

static char* get_str_state_mmc(STATES_MMC_ENUM state_mmc_crnt);

static char* get_str_state_mmc(STATES_MMC_ENUM state_mmc_crnt)
{
  static const char *state_mmc_str_init = "STATE_MMC_INIT";
  static const char *state_mmc_str_chal = "STATE_MMC_CHECK_HAL";
  static const char *state_mmc_str_cdev = "STATE_MMC_CHECK_DEV";
  static const char *state_mmc_str_erro = "STATE_MMC_ERROR";
  static const char *state_mmc_str_unkn = "STATE_MMC_UNKNOWN";
  static char *state_mmc_str;

  switch (state_mmc_crnt)
  {
    case (MMC_INIT):
      state_mmc_str = (char*) state_mmc_str_init;
      break;
    case (MMC_CHECK_HAL):
      state_mmc_str = (char*) state_mmc_str_chal;
      break;
    case (MMC_CHECK_DEV):
      state_mmc_str = (char*) state_mmc_str_cdev;
      break;
    case (MMC_ERROR):
      state_mmc_str = (char*) state_mmc_str_erro;
      break;
    default:
      state_mmc_str = (char*) state_mmc_str_unkn;
      break;
  }

  return state_mmc_str;
}

STATES_MMC_ENUM MMC5603NJ_init(I2C_HandleTypeDef *handle_i2c, UART_HandleTypeDef *handle_uart)
{
  static STATES_MMC_ENUM state_dev = MMC_INIT;
  static STATES_MMC_ENUM state_dev_was;
  static HAL_StatusTypeDef status_hal;

  state_dev_was = state_dev;

  switch (state_dev)
  {
    case (MMC_INIT):
      if ( handle_i2c == NULL )
      {
        state_dev = MMC_ERROR;
      }
      else
      {
        state_dev = MMC_CHECK_HAL;
      }
      break;
    case (MMC_CHECK_HAL):
      status_hal = HAL_I2C_IsDeviceReady(handle_i2c, MMC5603NJ_ADDR, 3U, 100U);
      if (status_hal == HAL_OK)
      {
        state_dev = MMC_CHECK_DEV;
      }
      else if (status_hal == HAL_TIMEOUT || status_hal == HAL_ERROR)
      {
        state_dev = MMC_ERROR;
      }
      break;
    case (MMC_CHECK_DEV):
      // TODO: read chip ID register
      state_dev = MMC_READY;
      break;
    case (MMC_ERROR):
    default:
      break;
  }

  if (handle_uart != NULL && state_dev != state_dev_was)
  {
    sprintf((char*) buffer_mmc, "[%lu]state_mmc=%s\r\n", HAL_GetTick(),
            get_str_state_mmc(state_dev));
    HAL_UART_Transmit(handle_uart, buffer_mmc, sizeof(buffer_mmc), 100U);
  }

  return state_dev;
}

