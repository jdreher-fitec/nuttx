/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-xiao/src/esp32s3_board_spi.c
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

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include "esp32s3-xiao.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_spi2_status
 *
 * Description:
 *   Report the status of the devices on the SPI2 bus.
 *
 *   SPI2 drives the onboard microSD slot of the XIAO ESP32S3 Sense.  The
 *   slot has no card detect line wired to the SoC, so the card is always
 *   reported as present and an absent card surfaces later as an MMC/SD
 *   initialization failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPI2

uint8_t esp32s3_spi2_status(struct spi_dev_s *dev, uint32_t devid)
{
  if (devid == SPIDEV_MMCSD(0))
    {
      return SPI_STATUS_PRESENT;
    }

  return 0;
}

#endif
