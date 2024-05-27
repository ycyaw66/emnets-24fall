/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cmath>

#include <errno.h>
#include <log.h>
#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "shell.h"

#include <xtimer.h>
#include "mpu6050.h"
using namespace std;
#define VCC_GPIO GPIO23

struct MPU6050Data
{
    float ax, ay, az;
    float gx, gy, gz;
};
#define g_acc (9.8)
int main(void)
{
    gpio_init(VCC_GPIO, GPIO_OUT);
    gpio_set(VCC_GPIO);
    xtimer_msleep(1000);
    MPU6050 mpu;
    uint8_t device_id = mpu.getDeviceID();
    uint16_t rate = mpu.getRate();
    mpu.initialize();
    mpu.setTempSensorEnabled(false);

    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
    uint16_t accel_fs_real = 1;
    float gyro_fs_convert = 1.0;
    if (gyro_fs == MPU6050_GYRO_FS_250)
        // gyro_fs_real = 250;
        gyro_fs_convert = 131.0;
    else if (gyro_fs == MPU6050_GYRO_FS_500)
        // gyro_fs_real = 500;
        gyro_fs_convert = 65.5;
    else if (gyro_fs == MPU6050_GYRO_FS_1000)
        // gyro_fs_real = 1000;
        gyro_fs_convert = 32.8;
    else if (gyro_fs == MPU6050_GYRO_FS_2000)
        // gyro_fs_real = 2000;
        gyro_fs_convert = 16.4;
    else
        LOG_ERROR("[main] Unknown GYRO_FS: 0x%x\n", gyro_fs);

    if (accel_fs_g == MPU6050_ACCEL_FS_2)
        accel_fs_real = g_acc * 2;
    else if (accel_fs_g == MPU6050_ACCEL_FS_4)
        accel_fs_real = g_acc * 4;
    else if (accel_fs_g == MPU6050_ACCEL_FS_8)
        accel_fs_real = g_acc * 8;
    else if (accel_fs_g == MPU6050_ACCEL_FS_16)
        accel_fs_real = g_acc * 16;
    else
        LOG_ERROR("[main] Unknown ACCEL_FS: 0x%x\n", accel_fs_g);
    float accel_fs_convert = 32768.0 / accel_fs_real;

    LOG_INFO("[main]DEVICE_ID:0x%x, RATE:0x%x\n", device_id, rate);
    uint8_t id = getDeviceID();
    printf("addr:[%x]\n", id);
    LOG_INFO("[main](X,Y,Z):(%d,%d,%d), (XG,YG,ZG):(%d,%d,%d)\n", mpu.getStandbyXAccelEnabled(), mpu.getStandbyYAccelEnabled(), mpu.getStandbyZAccelEnabled(),
        mpu.getStandbyXGyroEnabled(), mpu.getStandbyYGyroEnabled(), mpu.getStandbyZGyroEnabled());
    
    LOG_INFO("[main]temperature: %fC\n", mpu.getTemperature());


    int16_t ax, ay, az, gx, gy, gz;
    int i = 0;
    MPU6050Data data;
    while (1)
    {
        i += 1;
        /* code */
        xtimer_msleep(20);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        data.ax = ax / accel_fs_convert;
        data.ay = ay / accel_fs_convert;
        data.az = az / accel_fs_convert;
        data.gx = gx / gyro_fs_convert;
        data.gy = gy / gyro_fs_convert;
        data.gz = gz / gyro_fs_convert;
        LOG_INFO("[main]%d:%.02f:%.02f:%.02f:%.02f:%.02f:%.02f:\n", i, data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
    }
    return 0;
}
