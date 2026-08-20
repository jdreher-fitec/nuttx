/****************************************************************************
 * include/nuttx/video/ov3660.h
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
 * Portions of the register table ported from espressif/esp32-camera
 * (Apache-2.0 License).
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_OV3660_H
#define __INCLUDE_NUTTX_VIDEO_OV3660_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ov3660_initialize
 *
 * Description:
 *   Initialize the OV3660 camera sensor driver.
 *
 * Input Parameters:
 *   i2c    - I2C bus device
 *   width  - Desired frame width  (0 = QVGA 320)
 *   height - Desired frame height (0 = QVGA 240)
 *
 * Returned Value:
 *   Pointer to imgsensor_s on success; NULL on failure.
 *
 ****************************************************************************/

struct imgsensor_s *ov3660_initialize(struct i2c_master_s *i2c,
                                      uint16_t width,
                                      uint16_t height);

#endif /* __INCLUDE_NUTTX_VIDEO_OV3660_H */
