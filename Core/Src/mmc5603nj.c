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

#define MMC_ADDR 0x60U // Device I2C address shifted to the left by 1 bit.
#define MMC_NUM_REGS 22U
#define MMC_LSB 0.00625f

typedef enum
{
  XOUT0 = 0x00,
  XOUT1 = 0x01,
  YOUT0 = 0x02,
  YOUT1 = 0x03,
  ZOUT0 = 0x04,
  ZOUT1 = 0x05,
  XOUT2 = 0x06,
  YOUT2 = 0x07,
  ZOUT2 = 0x08,
  TOUT = 0x09,
  STATUS1 = 0x18,
  ODR = 0x1A,
  CTRL0 = 0x1B,
  CTRL1 = 0x1C,
  CTRL2 = 0x1D,
  STXTH = 0x1E,
  STYTH = 0x1F,
  STZTH = 0x20,
  STX = 0x27,
  STY = 0x28,
  STZ = 0x29,
  PID = 0x39
} REG_ADDRS_ENUM;

static uint8_t buf_diag_mmc[64];
static const REG_ADDRS_ENUM reg_addrs[MMC_NUM_REGS] = {
    XOUT0,
    XOUT1,
    YOUT0,
    YOUT1,
    ZOUT0,
    ZOUT1,
    XOUT2,
    YOUT2,
    ZOUT2,
    TOUT,
    STATUS1,
    ODR,
    CTRL0,
    CTRL1,
    CTRL2,
    STXTH,
    STYTH,
    STZTH,
    STX,
    STY,
    STZ,
    PID};

static bool check_reg_addr(uint8_t reg_addr);
static char *get_str_state_mmc(MMC5603NJ_STATES_ENUM state_mmc_current);
static bool read_registers(I2C_HandleTypeDef *handle_i2c,
                           uint8_t reg_addr,
                           uint8_t *buf_rx,
                           size_t buf_sz,
                           size_t num_regs);
static bool write_registers(I2C_HandleTypeDef *handle_i2c,
                            uint8_t reg_addr,
                            uint8_t *buf_tx,
                            size_t num_regs);

static bool check_reg_addr(uint8_t reg_addr)
{
  static bool is_valid;
  static uint8_t i;

  is_valid = false;

  for (i = 0; i < MMC_NUM_REGS; i++)
  {
    if (reg_addr == reg_addrs[i])
    {
      is_valid = true;
      break;
    }
  }

  return is_valid;
}

static char *get_str_state_mmc(MMC5603NJ_STATES_ENUM state_mmc_current)
{
  static const char *state_mmc_str_init = "STATE_MMC_INIT";
  static const char *state_mmc_str_check_hal = "STATE_MMC_CHECK_HAL";
  static const char *state_mmc_str_check_dev = "STATE_MMC_CHECK_DEV";
  static const char *state_mmc_str_ready = "STATE_MMC_READY";
  static const char *state_mmc_str_error = "STATE_MMC_ERROR";
  static const char *state_mmc_str_meas_init = "STATE_MMC_MEAS_INIT";
  static const char *state_mmc_str_meas_start = "STATE_MMC_MEAS_START";
  static const char *state_mmc_str_meas_wait = "STATE_MMC_MEAS_WAIT";
  static const char *state_mmc_str_meas_ready = "STATE_MMC_MEAS_READY";
  static const char *state_mmc_str_meas_done = "STATE_MMC_MEAS_DONE";
  static const char *state_mmc_str_unknown = "STATE_MMC_UNKNOWN";
  static char *state_mmc_str;

  switch (state_mmc_current)
  {
  case (MMC_INIT):
    state_mmc_str = (char *)state_mmc_str_init;
    break;
  case (MMC_CHECK_HAL):
    state_mmc_str = (char *)state_mmc_str_check_hal;
    break;
  case (MMC_CHECK_DEV):
    state_mmc_str = (char *)state_mmc_str_check_dev;
    break;
  case (MMC_READY):
    state_mmc_str = (char *)state_mmc_str_ready;
    break;
  case (MMC_ERROR):
    state_mmc_str = (char *)state_mmc_str_error;
    break;
  case (MMC_MEAS_INIT):
    state_mmc_str = (char *)state_mmc_str_meas_init;
    break;
  case (MMC_MEAS_START):
    state_mmc_str = (char *)state_mmc_str_meas_start;
    break;
  case (MMC_MEAS_WAIT):
    state_mmc_str = (char *)state_mmc_str_meas_wait;
    break;
  case (MMC_MEAS_READY):
    state_mmc_str = (char *)state_mmc_str_meas_ready;
    break;
  case (MMC_MEAS_DONE):
    state_mmc_str = (char *)state_mmc_str_meas_done;
    break;
  default:
    state_mmc_str = (char *)state_mmc_str_unknown;
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
        num_regs > MMC_NUM_REGS ||
        buf_sz < num_regs)
    {
      break;
    }

    buf_tx[0] = reg_addr;

    if (HAL_I2C_Master_Transmit(handle_i2c, MMC_ADDR, buf_tx, sizeof(buf_tx), 100U) != HAL_OK)
    {
      break;
    }

    if (HAL_I2C_Master_Receive(handle_i2c, MMC_ADDR, buf_rx, num_regs, 100U) != HAL_OK)
    {
      break;
    }

    mmc_read_status = true;
  } while (0);

  return mmc_read_status;
}

static bool write_registers(I2C_HandleTypeDef *handle_i2c,
                            uint8_t reg_addr,
                            uint8_t *buf_tx,
                            size_t num_regs)
{
  static bool mmc_write_status;

  mmc_write_status = false;

  do
  {
    if (handle_i2c == NULL || buf_tx == NULL)
    {
      break;
    }

    if (!check_reg_addr(reg_addr) ||
        num_regs == 0 ||
        num_regs > MMC_NUM_REGS)
    {
      break;
    }

    if (HAL_I2C_Master_Transmit(handle_i2c, MMC_ADDR, &reg_addr, 1U, 100U) != HAL_OK)
    {
      break;
    }

    if (HAL_I2C_Master_Transmit(handle_i2c, MMC_ADDR, buf_tx, num_regs, 100U) != HAL_OK)
    {
      break;
    }

    mmc_write_status = true;
  } while (0);

  return mmc_write_status;
}

MMC5603NJ_STATES_ENUM MMC5603NJ_init(I2C_HandleTypeDef *handle_i2c, UART_HandleTypeDef *handle_uart)
{
  static MMC5603NJ_STATES_ENUM state_init = MMC_INIT;
  static MMC5603NJ_STATES_ENUM state_init_was;

  state_init_was = state_init;

  switch (state_init)
  {
  case (MMC_INIT):
    if (handle_i2c == NULL)
    {
      state_init = MMC_ERROR;
    }
    else
    {
      state_init = MMC_CHECK_HAL;
    }
    break;
  case (MMC_CHECK_HAL):
    static HAL_StatusTypeDef status_hal;
    status_hal = HAL_I2C_IsDeviceReady(handle_i2c, MMC_ADDR, 3U, 100U);
    if (status_hal == HAL_OK)
    {
      state_init = MMC_CHECK_DEV;
    }
    else if (status_hal == HAL_TIMEOUT || status_hal == HAL_ERROR)
    {
      state_init = MMC_ERROR;
    }
    break;
  case (MMC_CHECK_DEV):
    static uint8_t reg_prod_id;
    if (read_registers(handle_i2c, 0x39U, &reg_prod_id, sizeof(reg_prod_id), 1U) && reg_prod_id == 0x10U)
    {
      state_init = MMC_READY;
    }
    else
    {
      state_init = MMC_ERROR;
    }
    break;
  case (MMC_READY):
  case (MMC_ERROR):
  default:
    break;
  }

  if (handle_uart != NULL && state_init != state_init_was)
  {
    snprintf((char *)buf_diag_mmc, sizeof(buf_diag_mmc), "[%lu]state_mmc=%s\r\n", HAL_GetTick(),
             get_str_state_mmc(state_init));
    HAL_UART_Transmit(handle_uart, buf_diag_mmc, sizeof(buf_diag_mmc), 100U);
  }

  return state_init;
}

MMC5603NJ_STATES_ENUM MMC5603NJ_measure(I2C_HandleTypeDef *handle_i2c, UART_HandleTypeDef *handle_uart, uint8_t *buf_ptr, size_t buf_sz)
{
  static MMC5603NJ_STATES_ENUM state_meas = MMC_MEAS_INIT;
  static MMC5603NJ_STATES_ENUM state_meas_was;

  state_meas_was = state_meas;

  switch (state_meas)
  {
  case (MMC_MEAS_INIT):
    if (handle_i2c == NULL || buf_ptr == NULL || buf_sz < MMC5603NJ_MEAS_REGS)
    {
      state_meas = MMC_ERROR;
    }
    else
    {
      state_meas = MMC_MEAS_START;
    }
    break;
  case (MMC_MEAS_START):
    static const uint8_t reg_ctrl0 = 0x01;
    if (write_registers(handle_i2c, CTRL0, (uint8_t *)&reg_ctrl0, 1U))
    {
      state_meas = MMC_MEAS_WAIT;
    }
    else
    {
      state_meas = MMC_ERROR;
    }
    break;
  case (MMC_MEAS_WAIT):
    static uint8_t reg_status1;
    if (read_registers(handle_i2c, STATUS1, &reg_status1, sizeof(reg_status1), 1U))
    {
      if ((reg_status1 & 0x40U) == 0x40U)
      {
        state_meas = MMC_MEAS_READY;
      }
    }
    else
    {
      state_meas = MMC_ERROR;
    }
    break;
  case (MMC_MEAS_READY):
    if (read_registers(handle_i2c, XOUT0, buf_ptr, buf_sz, MMC5603NJ_MEAS_REGS))
    {
      state_meas = MMC_MEAS_DONE;
    }
    else
    {
      state_meas = MMC_ERROR;
    }
    break;
  case (MMC_MEAS_DONE):
    state_meas = MMC_MEAS_START;
    break;
  case (MMC_ERROR):
  default:
    break;
  }

  if (handle_uart != NULL && state_meas != state_meas_was)
  {
    snprintf((char *)buf_diag_mmc, sizeof(buf_diag_mmc), "[%lu]state_mmc=%s\r\n", HAL_GetTick(),
             get_str_state_mmc(state_meas));
    HAL_UART_Transmit(handle_uart, buf_diag_mmc, sizeof(buf_diag_mmc), 100U);
  }

  return state_meas;
}

void MMC5603NJ_get_data(uint8_t *buf_ptr, size_t buf_sz, MMC5603NJ_DATA_STRUCT *data_ptr)
{
  static int32_t meas_x;
  static int32_t meas_y;
  static int32_t meas_z;

  if (buf_ptr == NULL || buf_sz != MMC5603NJ_MEAS_REGS || data_ptr == NULL)
  {
    return;
  }

  meas_x = (uint32_t)(*(buf_ptr + 0)) << 12 | (uint32_t)(*(buf_ptr + 1)) << 4 | (uint32_t)(*(buf_ptr + 6)) >> 4;
  meas_y = (uint32_t)(*(buf_ptr + 2)) << 12 | (uint32_t)(*(buf_ptr + 3)) << 4 | (uint32_t)(*(buf_ptr + 7)) >> 4;
  meas_z = (uint32_t)(*(buf_ptr + 4)) << 12 | (uint32_t)(*(buf_ptr + 5)) << 4 | (uint32_t)(*(buf_ptr + 8)) >> 4;

  meas_x -= (uint32_t)1 << 19;
  meas_y -= (uint32_t)1 << 19;
  meas_z -= (uint32_t)1 << 19;

  data_ptr->x = (float)meas_x * MMC_LSB;
  data_ptr->y = (float)meas_y * MMC_LSB;
  data_ptr->z = (float)meas_z * MMC_LSB;
}