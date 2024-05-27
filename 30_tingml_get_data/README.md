Expected result
===============
This test enables you to test all available low-level I2C functions using the shell.
Consult the `help` shell command for available actions.

Background
==========
Test for the low-level I2C driver.  Can be run manually or using python script interface.

Glossary
==========
DEV - The I2C device number, this is usually 0.
Consult the board header file for more information.</br>
ADDR - The address of the slave I2C device, this is usually the sensor.
Information on the slave address should be found in the sensor datasheet.</br>
REG - The register of the slave device/sensor.
Not all sensors follow this access method.
A I2C_NOSTOP (0x04) may be needed.</br>
LEN - The length to read or write.</br>
FLAG - Flags set for the I2C, more information is available in driver/include/periph/i2c.h

_note: Automated tests can be found in the
[RobotFW-tests](https://githubfast.com/RIOT-OS/RobotFW-tests/tree/master/tests/periph/i2c)
repository_

### MPU6050
目前使用的ESP32-wroom-32, 该板子有硬件的SDA 和 SCL引脚分别是GPIO21, GPIO22, 完成MPU6050 VCC, GND, SDA, SCL这四个引脚的连接。
之后执行代码，串口将持续打印3个轴的加速度和3个轴的角加速度。LED灯IN连接到ESP32的GPIO12引脚.
当设备移动动静较大，LED灯将闪烁10次。
```bash
sudo chmod 777 /dev/ttyUSB0
make BOARD=esp32-wroom-32 flash term
```