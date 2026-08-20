/****************************************************************************
 * drivers/video/ov3660.c
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
 * Portions ported from espressif/esp32-camera (Apache-2.0 License).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/arch.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV3660_I2C_ADDR         0x3c
#define OV3660_I2C_FREQ         100000

/* OV3660 Register Addresses (16-bit addressing) */

#define OV3660_REG_CHIP_ID_H    0x300a
#define OV3660_REG_CHIP_ID_L    0x300b
#define OV3660_CHIP_ID_VAL      0x3660

/* System control */

#define SYSTEM_CTROL0           0x3008
#define SYSTEM_CTROL0_RESET     0x82
#define SYSTEM_CTROL0_POWERDOWN 0x42

/* Output format control */

#define FORMAT_CTRL             0x501f
#define FORMAT_CTRL00           0x4300

/* Output formats */

#define FORMAT_CTRL_YUV422      0x00
#define FORMAT_CTRL_RGB565      0x01
#define FORMAT_CTRL00_YUYV      0x30
#define FORMAT_CTRL00_RGB565    0x61

/* ISP control */

#define ISP_CONTROL_01          0x5001
#define ISP_CONTROL_01_EN       0xa3

/* Timing control */

#define TIMING_TC_REG20         0x3820
#define TIMING_TC_REG21         0x3821

/* VGA / QVGA default sizes */

#define OV3660_QVGA_WIDTH       320
#define OV3660_QVGA_HEIGHT      240
#define OV3660_VGA_WIDTH        640
#define OV3660_VGA_HEIGHT       480
#define OV3660_SVGA_WIDTH       800
#define OV3660_SVGA_HEIGHT      600

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ov3660_reg_s
{
  uint16_t addr;
  uint16_t val;
};

struct ov3660_dev_s
{
  struct imgsensor_s sensor;
  struct i2c_master_s *i2c;
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
  struct v4l2_frmsizeenum frmsizes;
  bool streaming;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool ov3660_is_available(struct imgsensor_s *sensor);
static int ov3660_init(struct imgsensor_s *sensor);
static int ov3660_uninit(struct imgsensor_s *sensor);
static const char *ov3660_get_driver_name(struct imgsensor_s *sensor);
static int ov3660_validate_frame_setting(struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         imgsensor_format_t *datafmts,
                                         imgsensor_interval_t *interval);
static int ov3660_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval);
static int ov3660_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);
static int ov3660_get_supported_value(struct imgsensor_s *sensor,
                                      uint32_t id,
                                      imgsensor_supported_value_t *value);
static int ov3660_get_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t *value);
static int ov3660_set_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* OV3660 initialization register table (ported from espressif/esp32-camera)
 * This table uses special markers:
 *   REG_DLY (0xffff) = delay ms
 *   REGLIST_TAIL (0x0000) = end of table
 */

#define REG_DLY      0xffff
#define REGLIST_TAIL 0x0000

static const struct ov3660_reg_s g_ov3660_init_regs[] =
{
  { SYSTEM_CTROL0, 0x82 },  /* software reset */
  { REG_DLY, 10 },          /* delay 10ms */

  { 0x3103, 0x13 },
  { SYSTEM_CTROL0, 0x42 },
  { 0x3017, 0xff },
  { 0x3018, 0xff },
  { 0x302c, 0xc3 },         /* drive capability */
  { 0x4740, 0x21 },         /* clock pol control */

  { 0x3611, 0x01 },
  { 0x3612, 0x2d },

  { 0x3032, 0x00 },
  { 0x3614, 0x80 },
  { 0x3618, 0x00 },
  { 0x3619, 0x75 },
  { 0x3622, 0x80 },
  { 0x3623, 0x00 },
  { 0x3624, 0x03 },
  { 0x3630, 0x52 },
  { 0x3632, 0x07 },
  { 0x3633, 0xd2 },
  { 0x3704, 0x80 },
  { 0x3708, 0x66 },
  { 0x3709, 0x12 },
  { 0x370b, 0x12 },
  { 0x3717, 0x00 },
  { 0x371b, 0x60 },
  { 0x371c, 0x00 },
  { 0x3901, 0x13 },

  { 0x3600, 0x08 },
  { 0x3620, 0x43 },
  { 0x3702, 0x20 },
  { 0x3739, 0x48 },
  { 0x3730, 0x20 },
  { 0x370c, 0x0c },

  { 0x3a18, 0x00 },
  { 0x3a19, 0xf8 },

  { 0x3000, 0x10 },
  { 0x3004, 0xef },

  { 0x6700, 0x05 },
  { 0x6701, 0x19 },
  { 0x6702, 0xfd },
  { 0x6703, 0xd1 },
  { 0x6704, 0xff },
  { 0x6705, 0xff },

  { 0x3c01, 0x80 },
  { 0x3c00, 0x04 },
  { 0x3a08, 0x00 }, { 0x3a09, 0x62 }, /* 50Hz Band Width Step (10bit) */
  { 0x3a0e, 0x08 }, /* 50Hz Max Bands in One Frame (6 bit) */
  { 0x3a0a, 0x00 }, { 0x3a0b, 0x52 }, /* 60Hz Band Width Step (10bit) */
  { 0x3a0d, 0x09 }, /* 60Hz Max Bands in One Frame (6 bit) */

  { 0x3a00, 0x3a },         /* night mode off */
  { 0x3a14, 0x09 },
  { 0x3a15, 0x30 },
  { 0x3a02, 0x09 },
  { 0x3a03, 0x30 },

  { 0x440e, 0x08 },         /* compression ctrl */
  { 0x4520, 0x0b },
  { 0x460b, 0x37 },
  { 0x4713, 0x02 },
  { 0x471c, 0xd0 },
  { 0x5086, 0x00 },

  { 0x5002, 0x00 },
  { FORMAT_CTRL, 0x00 },

  { SYSTEM_CTROL0, 0x02 },

  { 0x5180, 0xff },
  { 0x5181, 0xf2 },
  { 0x5182, 0x00 },
  { 0x5183, 0x14 },
  { 0x5184, 0x25 },
  { 0x5185, 0x24 },
  { 0x5186, 0x16 },
  { 0x5187, 0x16 },
  { 0x5188, 0x16 },
  { 0x5189, 0x68 },
  { 0x518a, 0x60 },
  { 0x518b, 0xe0 },
  { 0x518c, 0xb2 },
  { 0x518d, 0x42 },
  { 0x518e, 0x35 },
  { 0x518f, 0x56 },
  { 0x5190, 0x56 },
  { 0x5191, 0xf8 },
  { 0x5192, 0x04 },
  { 0x5193, 0x70 },
  { 0x5194, 0xf0 },
  { 0x5195, 0xf0 },
  { 0x5196, 0x03 },
  { 0x5197, 0x01 },
  { 0x5198, 0x04 },
  { 0x5199, 0x12 },
  { 0x519a, 0x04 },
  { 0x519b, 0x00 },
  { 0x519c, 0x06 },
  { 0x519d, 0x82 },
  { 0x519e, 0x38 },

  { 0x5381, 0x1d },
  { 0x5382, 0x60 },
  { 0x5383, 0x03 },
  { 0x5384, 0x0c },
  { 0x5385, 0x78 },
  { 0x5386, 0x84 },
  { 0x5387, 0x7d },
  { 0x5388, 0x6b },
  { 0x5389, 0x12 },
  { 0x538a, 0x01 },
  { 0x538b, 0x98 },

  { 0x5480, 0x01 },

  { 0x5000, 0xa7 },
  { 0x5800, 0x0C },
  { 0x5801, 0x09 },
  { 0x5802, 0x0C },
  { 0x5803, 0x0C },
  { 0x5804, 0x0D },
  { 0x5805, 0x17 },
  { 0x5806, 0x06 },
  { 0x5807, 0x05 },
  { 0x5808, 0x04 },
  { 0x5809, 0x06 },
  { 0x580a, 0x09 },
  { 0x580b, 0x0E },
  { 0x580c, 0x05 },
  { 0x580d, 0x01 },
  { 0x580e, 0x01 },
  { 0x580f, 0x01 },
  { 0x5810, 0x05 },
  { 0x5811, 0x0D },
  { 0x5812, 0x05 },
  { 0x5813, 0x01 },
  { 0x5814, 0x01 },
  { 0x5815, 0x01 },
  { 0x5816, 0x05 },
  { 0x5817, 0x0D },
  { 0x5818, 0x08 },
  { 0x5819, 0x06 },
  { 0x581a, 0x05 },
  { 0x581b, 0x07 },
  { 0x581c, 0x0B },
  { 0x581d, 0x0D },
  { 0x581e, 0x12 },
  { 0x581f, 0x0D },
  { 0x5820, 0x0E },
  { 0x5821, 0x10 },
  { 0x5822, 0x10 },
  { 0x5823, 0x1E },
  { 0x5824, 0x53 },
  { 0x5825, 0x15 },
  { 0x5826, 0x05 },
  { 0x5827, 0x14 },
  { 0x5828, 0x54 },
  { 0x5829, 0x25 },
  { 0x582a, 0x33 },
  { 0x582b, 0x33 },
  { 0x582c, 0x34 },
  { 0x582d, 0x16 },
  { 0x582e, 0x24 },
  { 0x582f, 0x41 },
  { 0x5830, 0x50 },
  { 0x5831, 0x42 },
  { 0x5832, 0x15 },
  { 0x5833, 0x25 },
  { 0x5834, 0x34 },
  { 0x5835, 0x33 },
  { 0x5836, 0x24 },
  { 0x5837, 0x26 },
  { 0x5838, 0x54 },
  { 0x5839, 0x25 },
  { 0x583a, 0x15 },
  { 0x583b, 0x25 },
  { 0x583c, 0x53 },
  { 0x583d, 0xCF },

  { 0x3a0f, 0x30 },
  { 0x3a10, 0x28 },
  { 0x3a1b, 0x30 },
  { 0x3a1e, 0x28 },
  { 0x3a11, 0x60 },
  { 0x3a1f, 0x14 },

  { 0x5302, 0x28 },
  { 0x5303, 0x20 },

  { 0x5306, 0x1c }, /* de-noise offset 1 */
  { 0x5307, 0x28 }, /* de-noise offset 2 */

  { 0x4002, 0xc5 },
  { 0x4003, 0x81 },
  { 0x4005, 0x12 },

  { 0x5688, 0x11 },
  { 0x5689, 0x11 },
  { 0x568a, 0x11 },
  { 0x568b, 0x11 },
  { 0x568c, 0x11 },
  { 0x568d, 0x11 },
  { 0x568e, 0x11 },
  { 0x568f, 0x11 },

  { 0x5580, 0x06 },
  { 0x5588, 0x00 },
  { 0x5583, 0x40 },
  { 0x5584, 0x2c },

  { ISP_CONTROL_01, 0x83 }, /* turn color matrix, awb and SDE */
  { REGLIST_TAIL, 0x00 },   /* tail */
};

/* YUV422 (YUYV) format register settings */

static const struct ov3660_reg_s g_ov3660_fmt_yuv422[] =
{
  { FORMAT_CTRL, FORMAT_CTRL_YUV422 },
  { FORMAT_CTRL00, FORMAT_CTRL00_YUYV },
  { REGLIST_TAIL, 0x00 },
};

/* RGB565 format register settings */

static const struct ov3660_reg_s g_ov3660_fmt_rgb565[] =
{
  { FORMAT_CTRL, FORMAT_CTRL_RGB565 },
  { FORMAT_CTRL00, FORMAT_CTRL00_RGB565 },
  { REGLIST_TAIL, 0x00 },
};

static const struct imgsensor_ops_s g_ov3660_ops =
{
  .is_available           = ov3660_is_available,
  .init                   = ov3660_init,
  .uninit                 = ov3660_uninit,
  .get_driver_name        = ov3660_get_driver_name,
  .validate_frame_setting = ov3660_validate_frame_setting,
  .start_capture          = ov3660_start_capture,
  .stop_capture           = ov3660_stop_capture,
  .get_supported_value    = ov3660_get_supported_value,
  .get_value              = ov3660_get_value,
  .set_value              = ov3660_set_value,
};

static const struct v4l2_fmtdesc g_ov3660_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_YUYV,
    .description = "YUV 4:2:2 (YUYV)",
  },
  {
    .pixelformat = V4L2_PIX_FMT_RGB565,
    .description = "RGB565",
  },
};

static const struct v4l2_frmivalenum g_ov3660_frmintervals[] =
{
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete =
      {
        .numerator   = 1,
        .denominator = 30,
      },
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov3660_putreg
 *
 * Description:
 *   Write an 8-bit value to an OV3660 register via 16-bit addressing.
 *   OV3660 uses 16-bit register addresses (high byte first, then low byte).
 *
 ****************************************************************************/

static int ov3660_putreg(struct i2c_master_s *i2c,
                         uint16_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t buf[3];
  int ret;

  buf[0] = (regaddr >> 8) & 0xff;   /* High byte of address */
  buf[1] = regaddr & 0xff;          /* Low byte of address */
  buf[2] = regval;                  /* Value */

  msg.frequency = OV3660_I2C_FREQ;
  msg.addr      = OV3660_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 3;

  ret = I2C_TRANSFER(i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C write to 0x%04x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ov3660_getreg
 *
 * Description:
 *   Read an 8-bit value from an OV3660 register via 16-bit addressing.
 *
 ****************************************************************************/

static int ov3660_getreg(struct i2c_master_s *i2c,
                         uint16_t regaddr, uint8_t *regval)
{
  struct i2c_msg_s msg[2];
  uint8_t addr_buf[2];
  int ret;

  addr_buf[0] = (regaddr >> 8) & 0xff;
  addr_buf[1] = regaddr & 0xff;

  msg[0].frequency = OV3660_I2C_FREQ;
  msg[0].addr      = OV3660_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = addr_buf;
  msg[0].length    = 2;

  msg[1].frequency = OV3660_I2C_FREQ;
  msg[1].addr      = OV3660_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C read from 0x%04x failed: %d\n", regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ov3660_modreg
 *
 * Description:
 *   Read-modify-write an 8-bit register.
 *
 ****************************************************************************/

static int ov3660_modreg(struct i2c_master_s *i2c,
                         uint16_t regaddr,
                         uint8_t clearbits, uint8_t setbits)
{
  uint8_t regval;
  int ret;

  ret = ov3660_getreg(i2c, regaddr, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = (regval & ~clearbits) | setbits;

  return ov3660_putreg(i2c, regaddr, regval);
}

/****************************************************************************
 * Name: ov3660_putreglist
 *
 * Description:
 *   Write a list of registers, handling REG_DLY (delay) markers.
 *
 ****************************************************************************/

static int ov3660_putreglist(struct i2c_master_s *i2c,
                             const struct ov3660_reg_s *reglist)
{
  int ret;

  for (int i = 0; ; i++)
    {
      if (reglist[i].addr == REGLIST_TAIL)
        {
          break;
        }

      if (reglist[i].addr == REG_DLY)
        {
          up_mdelay(reglist[i].val);
          continue;
        }

      ret = ov3660_putreg(i2c, reglist[i].addr, (uint8_t)reglist[i].val);
      if (ret < 0)
        {
          snerr("OV3660 write[%d] 0x%04x=0x%02x FAILED: %d\n",
                i, reglist[i].addr, (uint8_t)reglist[i].val, ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ov3660_is_available
 *
 * Description:
 *   Probe for the OV3660 camera sensor by reading the chip ID.
 *   Chip ID is split across two 8-bit registers:
 *   - 0x300A contains high byte (should be 0x36)
 *   - 0x300B contains low byte (should be 0x60)
 *
 ****************************************************************************/

static bool ov3660_is_available(struct imgsensor_s *sensor)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;
  uint8_t id_h = 0;
  uint8_t id_l = 0;
  uint16_t chip_id;
  int ret;

  ret = ov3660_getreg(priv->i2c, OV3660_REG_CHIP_ID_H, &id_h);
  if (ret < 0)
    {
      return false;
    }

  ret = ov3660_getreg(priv->i2c, OV3660_REG_CHIP_ID_L, &id_l);
  if (ret < 0)
    {
      return false;
    }

  chip_id = (id_h << 8) | id_l;
  sninfo("OV3660 chip ID: 0x%04x (expected 0x%04x)\n",
         chip_id, OV3660_CHIP_ID_VAL);

  return (chip_id == OV3660_CHIP_ID_VAL);
}

/****************************************************************************
 * Name: ov3660_init
 *
 * Description:
 *   Initialize the OV3660 camera sensor.
 *
 ****************************************************************************/

static int ov3660_init(struct imgsensor_s *sensor)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;
  uint8_t id_h = 0;
  uint8_t id_l = 0;
  int ret;

  /* Software reset */

  ret = ov3660_putreg(priv->i2c, SYSTEM_CTROL0, SYSTEM_CTROL0_RESET);
  if (ret < 0)
    {
      snerr("OV3660 soft reset failed: %d\n", ret);
      return ret;
    }

  up_mdelay(100);

  /* Write vendor initialization register table */

  ret = ov3660_putreglist(priv->i2c, g_ov3660_init_regs);
  if (ret < 0)
    {
      snerr("OV3660 init regs failed: %d\n", ret);
      return ret;
    }

  up_mdelay(100);

  /* Set default format to YUV422 (YUYV) */

  ret = ov3660_putreglist(priv->i2c, g_ov3660_fmt_yuv422);
  if (ret < 0)
    {
      snerr("OV3660 format setup failed: %d\n", ret);
      return ret;
    }

  priv->pixelformat = IMGSENSOR_PIX_FMT_YUYV;

  /* Debug: verify chip ID after init */

  ov3660_getreg(priv->i2c, OV3660_REG_CHIP_ID_H, &id_h);
  ov3660_getreg(priv->i2c, OV3660_REG_CHIP_ID_L, &id_l);
  sninfo("OV3660 init done: id=0x%02x%02x ret=%d\n",
         id_h, id_l, ret);

  return ret;
}

/****************************************************************************
 * Name: ov3660_uninit
 *
 * Description:
 *   Uninitialize the OV3660 camera sensor.
 *
 ****************************************************************************/

static int ov3660_uninit(struct imgsensor_s *sensor)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  /* Soft reset */

  ov3660_putreg(priv->i2c, SYSTEM_CTROL0, SYSTEM_CTROL0_RESET);
  priv->streaming = false;

  return OK;
}

/****************************************************************************
 * Name: ov3660_get_driver_name
 *
 * Description:
 *   Return the driver name.
 *
 ****************************************************************************/

static const char *ov3660_get_driver_name(struct imgsensor_s *sensor)
{
  return "OV3660";
}

/****************************************************************************
 * Name: ov3660_validate_frame_setting
 *
 * Description:
 *   Validate the requested frame settings match the configured sensor.
 *
 ****************************************************************************/

static int ov3660_validate_frame_setting(struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmts,
                                         imgsensor_format_t *datafmts,
                                         imgsensor_interval_t *interval)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  if (nr_datafmts < 1 || !datafmts)
    {
      return -EINVAL;
    }

  /* Only YUYV and RGB565 formats are supported */

  if (datafmts[IMGSENSOR_FMT_MAIN].pixelformat != IMGSENSOR_PIX_FMT_YUYV &&
      datafmts[IMGSENSOR_FMT_MAIN].pixelformat != IMGSENSOR_PIX_FMT_RGB565)
    {
      return -EINVAL;
    }

  /* Validate resolution matches configured size */

  if (datafmts[IMGSENSOR_FMT_MAIN].width != priv->width ||
      datafmts[IMGSENSOR_FMT_MAIN].height != priv->height)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: ov3660_start_capture
 *
 * Description:
 *   Start camera capture with the requested settings.
 *
 ****************************************************************************/

static int ov3660_start_capture(struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                imgsensor_format_t *datafmts,
                                imgsensor_interval_t *interval)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;
  int ret;

  if (priv->streaming)
    {
      return -EBUSY;
    }

  /* Configure output format based on requested pixel format */

  if (datafmts[IMGSENSOR_FMT_MAIN].pixelformat == IMGSENSOR_PIX_FMT_RGB565)
    {
      ret = ov3660_putreglist(priv->i2c, g_ov3660_fmt_rgb565);
      if (ret < 0)
        {
          return ret;
        }
      priv->pixelformat = IMGSENSOR_PIX_FMT_RGB565;
    }
  else
    {
      ret = ov3660_putreglist(priv->i2c, g_ov3660_fmt_yuv422);
      if (ret < 0)
        {
          return ret;
        }
      priv->pixelformat = IMGSENSOR_PIX_FMT_YUYV;
    }

  priv->streaming = true;
  return OK;
}

/****************************************************************************
 * Name: ov3660_stop_capture
 *
 * Description:
 *   Stop camera capture.
 *
 ****************************************************************************/

static int ov3660_stop_capture(struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  priv->streaming = false;
  return OK;
}

/****************************************************************************
 * Name: ov3660_get_supported_value
 *
 * Description:
 *   Get supported range/values for a given control ID.
 *
 ****************************************************************************/

static int ov3660_get_supported_value(struct imgsensor_s *sensor,
                                      uint32_t id,
                                      imgsensor_supported_value_t *value)
{
  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
        value->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        value->u.range.minimum       = 0;
        value->u.range.maximum       = 1;
        value->u.range.step          = 1;
        value->u.range.default_value = 0;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: ov3660_get_value
 *
 * Description:
 *   Get the current value of a control.
 *
 ****************************************************************************/

static int ov3660_get_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t *value)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;
  uint8_t regval;
  int ret;

  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
        ret = ov3660_getreg(priv->i2c, TIMING_TC_REG21, &regval);
        if (ret < 0)
          {
            return ret;
          }

        value->value32 = (regval & 0x06) ? 1 : 0;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: ov3660_set_value
 *
 * Description:
 *   Set the value of a control.
 *
 ****************************************************************************/

static int ov3660_set_value(struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  struct ov3660_dev_s *priv = (struct ov3660_dev_s *)sensor;

  switch (id)
    {
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
        return ov3660_modreg(priv->i2c, TIMING_TC_REG21,
                             0x06,
                             value.value32 ? 0x06 : 0x00);

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
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
                                      uint16_t height)
{
  struct ov3660_dev_s *priv;

  if (!i2c)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct ov3660_dev_s));
  if (!priv)
    {
      return NULL;
    }

  priv->i2c       = i2c;
  priv->streaming = false;
  priv->width     = width  ? width  : OV3660_QVGA_WIDTH;
  priv->height    = height ? height : OV3660_QVGA_HEIGHT;

  priv->frmsizes.type             = V4L2_FRMSIZE_TYPE_DISCRETE;
  priv->frmsizes.discrete.width   = priv->width;
  priv->frmsizes.discrete.height  = priv->height;

  priv->sensor.ops              = &g_ov3660_ops;
  priv->sensor.fmtdescs         = g_ov3660_fmtdescs;
  priv->sensor.fmtdescs_num     = nitems(g_ov3660_fmtdescs);
  priv->sensor.frmsizes         = &priv->frmsizes;
  priv->sensor.frmsizes_num     = 1;
  priv->sensor.frmintervals     = g_ov3660_frmintervals;
  priv->sensor.frmintervals_num = nitems(g_ov3660_frmintervals);

  return &priv->sensor;
}
