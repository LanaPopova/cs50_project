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
#include <stdbool.h>
#include <stdio.h>

#define MMC5603NJ_ADDR 0x60U // Device I2C address shifted to the left by 1 bit.
#define MMC5603NJ_NUM_REGS 22U

static uint8_t buffer_mmc[64] = {0};
static uint8_t reg_addrs[MMC5603NJ_NUM_REGS] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                0x08, 0x09, 0x18, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
                                                0x1F, 0x20, 0x27, 0x28, 0x29, 0x39};

static bool check_reg_addr(uint8_t reg_addr);
static char *get_str_state_mmc(STATES_MMC_ENUM state_mmc_crnt);
static bool read_registers(I2C_HandleTypeDef *handle_i2c,
                           uint8_t reg_addr,
                           uint8_t *buf_rx,
                           size_t buf_sz,
                           size_t num_regs);

static bool check_reg_addr(uint8_t reg_addr)
{
  static bool is_valid;
  static uint8_t i;

  is_valid = false;

  for (i = 0; i < MMC5603NJ_NUM_REGS; i++)
  {
    if (reg_addr == reg_addrs[i])
    {
      is_valid = true;
      break;
    }
  }

  return is_valid;
}

static char *get_str_state_mmc(STATES_MMC_ENUM state_mmc_crnt)
{
  static const char *state_mmc_str_init = "STATE_MMC_INIT";
  static const char *state_mmc_str_chal = "STATE_MMC_CHECK_HAL";
  static const char *state_mmc_str_cdev = "STATE_MMC_CHECK_DEV";
  static const char *state_mmc_str_redy = "STATE_MMC_READY";
  static const char *state_mmc_str_erro = "STATE_MMC_ERROR";
  static const char *state_mmc_str_unkn = "STATE_MMC_UNKNOWN";
  static char *state_mmc_str;

  switch (state_mmc_crnt)
  {
  case (MMC_INIT):
    state_mmc_str = (char *)state_mmc_str_init;
    break;
  case (MMC_CHECK_HAL):
    state_mmc_str = (char *)state_mmc_str_chal;
    break;
  case (MMC_CHECK_DEV):
    state_mmc_str = (char *)state_mmc_str_cdev;
    break;
  case (MMC_READY):
    state_mmc_str = (char *)state_mmc_str_redy;
    break;
  case (MMC_ERROR):
    state_mmc_str = (char *)state_mmc_str_erro;
    break;
  default:
    state_mmc_str = (char *)state_mmc_str_unkn;
    break;
  }

  return state_mmc_str;
}

static bool read_registers(I2C_HandleTypeDef *handle_i2c,
                           uint8_t reg_addr,
                           uint8_t *buf_rx,
                           size_t buf_sz,
                           size_t num_regs)
{
  static bool mmc_read_status;
  static uint8_t buf_tx[1];

  mmc_read_status = false;

  do
  {
    if (handle_i2c == NULL || buf_rx == NULL)
    {
      break;
    }

    if (!check_reg_addr(reg_addr) ||
        num_regs == 0 ||
        num_regs > MMC5603NJ_NUM_REGS ||
        buf_sz < num_regs)
    {
      break;
    }

    buf_tx[0] = reg_addr;

    if (HAL_I2C_Master_Transmit(handle_i2c, MMC5603NJ_ADDR, buf_tx, sizeof(buf_tx), 100U) != HAL_OK)
    {
      break;
    }

    if (HAL_I2C_Master_Receive(handle_i2c, MMC5603NJ_ADDR, buf_rx, num_regs, 100U) != HAL_OK)
    {
      break;
    }

    mmc_read_status = true;
  } while (0);

  return mmc_read_status;
}

STATES_MMC_ENUM MMC5603NJ_init(I2C_HandleTypeDef *handle_i2c, UART_HandleTypeDef *handle_uart)
{
  static STATES_MMC_ENUM state_dev = MMC_INIT;
  static STATES_MMC_ENUM state_dev_was;

  state_dev_was = state_dev;

  switch (state_dev)
  {
  case (MMC_INIT):
    if (handle_i2c == NULL)
    {
      state_dev = MMC_ERROR;
    }
    else
    {
      state_dev = MMC_CHECK_HAL;
    }
    break;
  case (MMC_CHECK_HAL):
    static HAL_StatusTypeDef status_hal;
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
    uint8_t reg_prod_id;
    if (read_registers(handle_i2c, 0x39U, &reg_prod_id, sizeof(reg_prod_id), 1U) && reg_prod_id == 0x10U)
    {
      state_dev = MMC_READY;
    }
    else
    {
      state_dev = MMC_ERROR;
    }
    break;
  case (MMC_ERROR):
  default:
    break;
  }

  if (handle_uart != NULL && state_dev != state_dev_was)
  {
    snprintf((char *)buffer_mmc, sizeof(buffer_mmc), "[%lu]state_mmc=%s\r\n", HAL_GetTick(),
             get_str_state_mmc(state_dev));
    HAL_UART_Transmit(handle_uart, buffer_mmc, sizeof(buffer_mmc), 100U);
  }

  return state_dev;
}
