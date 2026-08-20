/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-xiao/src/esp32s3_board_imu.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm6dsl.h>

#include "esp32s3_i2c.h"
#include "esp32s3-xiao.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The IMU sits on the expansion board, which plugs into the XIAO's own I2C
 * pads: SDA on GPIO5 (D4) and SCL on GPIO6 (D5).  That is a different bus
 * from the camera's SCCB lines, so the IMU uses I2C1 while the camera keeps
 * I2C0.  The pins themselves come from CONFIG_ESP32S3_I2C1_SDAPIN/SCLPIN.
 */

#define IMU_I2C_PORT   1

#define IMU_ACCEL_DEVPATH  "/dev/imu0"
#define IMU_GYRO_DEVPATH   "/dev/gyro0"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_imu_initialize
 *
 * Description:
 *   Register the 6-axis IMU carried by the XIAOML expansion board.
 *
 *   The part is an LSM6DS3TR-C, which is register compatible with the
 *   LSM6DSL this driver was written for and reports the same WHO_AM_I value
 *   (0x6a), so the existing driver drives it unchanged.
 *
 *   Accelerometer and gyroscope are one device on the bus, both started
 *   together, and are exposed as two character devices reading the two
 *   output register banks.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32s3_imu_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = esp32s3_i2cbus_initialize(IMU_I2C_PORT);
  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C%d\n", IMU_I2C_PORT);
      return -ENODEV;
    }

  ret = lsm6dsl_sensor_register(IMU_ACCEL_DEVPATH, i2c, LSM6DSLACCEL_ADDR0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register accelerometer at %s: %d\n",
            IMU_ACCEL_DEVPATH, ret);
      return ret;
    }

  ret = lsm6dsl_sensor_register_gyro(IMU_GYRO_DEVPATH, i2c,
                                     LSM6DSLACCEL_ADDR0);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register gyroscope at %s: %d\n",
            IMU_GYRO_DEVPATH, ret);
      return ret;
    }

  sninfo("IMU registered at %s and %s\n", IMU_ACCEL_DEVPATH,
         IMU_GYRO_DEVPATH);
  return OK;
}
