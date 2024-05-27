/*
 * Copyright (C) 2019 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 *
 * @file
 * @brief       TensorFlow Lite test application
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

/* Provide an Arduino-Like API to be able to easily reuse the code from
   upstream examples */
#include "mpu6050.h"
#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "shell.h"
#include <log.h>
#include <xtimer.h>
#include "ledcontroller.hh"
void setup();
int predict(float *imu_data, int data_len);
using namespace std;
#define VCC_GPIO GPIO23
#define g_acc (9.8)
#define SAMPLES_PER_GESTURE (50)
MPU6050 mpu;
float gyro_fs_convert = 1.0;
float accel_fs_convert;
float imu_data[SAMPLES_PER_GESTURE * 6] = {0};
int data_len = SAMPLES_PER_GESTURE * 6;
void init_mpu(void){
    uint8_t device_id = mpu.getDeviceID();
    uint16_t rate = mpu.getRate();
    mpu.initialize();
    mpu.setTempSensorEnabled(false);
    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
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
    uint16_t accel_fs_real = 1;

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
    accel_fs_convert = 32768.0 / accel_fs_real;
    LOG_INFO("[main]DEVICE_ID:0x%x, RATE:0x%x\n", device_id, rate);
    uint8_t id = getDeviceID();
    printf("addr:[%x]\n", id);
}
void get_imu_data(void){
    int16_t ax, ay, az, gx, gy, gz;
    for(int i = 0; i < SAMPLES_PER_GESTURE; ++i)
    {
        i += 1;
        /* code */
        xtimer_msleep(20);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        imu_data[i*6 + 0] = ax / accel_fs_convert;
        imu_data[i*6 + 1] = ay / accel_fs_convert;
        imu_data[i*6 + 2] = az / accel_fs_convert;
        imu_data[i*6 + 3] = gx / gyro_fs_convert;
        imu_data[i*6 + 4] = gy / gyro_fs_convert;
        imu_data[i*6 + 5] = gz / gyro_fs_convert;
        // LOG_INFO("[main]%d:%.02f:%.02f:%.02f:%.02f:%.02f:%.02f:\n", i, imu_data[i*6+0], imu_data[i*6+1], imu_data[i*6+2], imu_data[i*6+3], imu_data[i*6+4], imu_data[i*6+5]);
    }

} 


int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    gpio_init(VCC_GPIO, GPIO_OUT);
    gpio_set(VCC_GPIO);
    gpio_init(GPIO14, GPIO_OUT);
    gpio_set(GPIO14);
    xtimer_msleep(1000);
    init_mpu();  
    setup();
    LEDController led(GPIO12);    
    while (true)
    {
      get_imu_data();
      int res = predict(imu_data, data_len);
      LOG_INFO("RES: %d\n", res);

      switch (res)
      {
        case 0:
            led.change_led_state(0);
            break;
        case 1:
            led.change_led_state(1);
            break;
        case 2:
            led.blink_faster(4, 100);
            break;
        case 3:
            led.blink_faster(2, 200);
            break;        
        default:
            break;
      }
      /* code */
    }
    
}
