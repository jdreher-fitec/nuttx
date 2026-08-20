/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-xiao/src/esp32s3_board_camera.c
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

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/v4l2_cap.h>

#ifdef CONFIG_VIDEO_OV5640
#  include <nuttx/video/ov5640.h>
#endif

#ifdef CONFIG_VIDEO_OV3660
#  include <nuttx/video/ov3660.h>
#endif

#include "esp32s3_i2c.h"
#include "esp32s3_cam.h"
#include "esp32s3-xiao.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The camera SCCB bus is wired to I2C0 (SIOD/GPIO40, SIOC/GPIO39).  The pins
 * themselves are selected through CONFIG_ESP32S3_I2C0_SDAPIN/SCLPIN.
 */

#define CAMERA_I2C_PORT   0

/* Default capture geometry.  QVGA keeps a frame small enough for the fixed
 * size DMA transfers implemented by esp32s3_cam.
 */

#define CAMERA_WIDTH      320
#define CAMERA_HEIGHT     240

/* Time for XCLK to stabilise before the sensor answers on SCCB */

#define CAMERA_XCLK_DELAY 10

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_camera_initialize
 *
 * Description:
 *   Bring up the camera pipeline of the XIAO ESP32S3 Sense:
 *
 *   1. Register the ESP32-S3 CAM (DVP) imgdata driver, which also starts the
 *      XCLK output the sensor needs before it will answer on SCCB.
 *   2. Probe and register every enabled sensor driver.
 *
 *   Both OV5640 and OV3660 may be registered at the same time.  The capture
 *   framework calls IMGSENSOR_IS_AVAILABLE() on each registered sensor and
 *   binds to the first one that acknowledges its chip ID over SCCB, so a
 *   single build runs on either camera module.
 *
 *   The XIAO ESP32S3 Sense leaves PWDN and RESET unconnected, so unlike other
 *   ESP32-S3 camera boards there is no power sequencing to perform here.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32s3_camera_initialize(void)
{
  struct imgdata_s *imgdata;
  struct imgsensor_s *imgsensor;
  struct i2c_master_s *i2c;
  int registered = 0;
  int ret;

  /* Register the DVP controller.  This starts XCLK. */

  imgdata = esp32s3_cam_initialize();
  if (imgdata == NULL)
    {
      snerr("ERROR: Failed to initialize ESP32-S3 CAM\n");
      return -ENODEV;
    }

  imgdata_register(imgdata);

  /* Give XCLK time to settle before talking to the sensor */

  up_mdelay(CAMERA_XCLK_DELAY);

  i2c = esp32s3_i2cbus_initialize(CAMERA_I2C_PORT);
  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C%d\n", CAMERA_I2C_PORT);
      return -ENODEV;
    }

#ifdef CONFIG_VIDEO_OV5640
  imgsensor = ov5640_initialize(i2c, CAMERA_WIDTH, CAMERA_HEIGHT);
  if (imgsensor == NULL)
    {
      snerr("ERROR: Failed to initialize OV5640\n");
    }
  else
    {
      ret = imgsensor_register(imgsensor);
      if (ret < 0)
        {
          snerr("ERROR: Failed to register OV5640: %d\n", ret);
        }
      else
        {
          registered++;
        }
    }
#endif

#ifdef CONFIG_VIDEO_OV3660
  imgsensor = ov3660_initialize(i2c, CAMERA_WIDTH, CAMERA_HEIGHT);
  if (imgsensor == NULL)
    {
      snerr("ERROR: Failed to initialize OV3660\n");
    }
  else
    {
      ret = imgsensor_register(imgsensor);
      if (ret < 0)
        {
          snerr("ERROR: Failed to register OV3660: %d\n", ret);
        }
      else
        {
          registered++;
        }
    }
#endif

  if (registered == 0)
    {
      snerr("ERROR: No camera sensor driver could be registered\n");
      return -ENODEV;
    }

  sninfo("Camera initialized: %d sensor driver(s) registered\n", registered);
  return OK;
}
