## 实验一: 基础设备控制

基础设备控制实验着眼于RIOT的多线程控制，基于GPIO引脚输出的LED灯控制以及基于I2C总线的IMU传感器数据读取。最终目标是根据设备的姿态展示不同的LED显示状态。

### 实验目标：

- 理解RIOT操作系统的多线程控制原理和方法。
- 掌握基于GPIO引脚的LED灯控制技术。
- 学习通过I2C总线读取IMU传感器数据的方法。
- 实现根据设备姿态展示不同LED显示状态的功能。

### RIOT-OS 多线程使用


```bash

```
### LED灯控制
<img src="./figs/led.jpeg" width="200" height="200">

LED灯只有GND、VCC和IN这三个引脚，GND和VCC接到对应的引脚即可(后续如果引脚不够，可以将GND接到任意低电压的引脚，VCC接到高电压的引脚), IN引脚接到任意一个GPIO引脚，如`GPIO12`引脚。

请浏览`03_threads_led/ledcontroller.cpp`和`03_threads_led/ledcontroller.hh`文件的要求并在对应的函数补充代码实现灯控制和自定义闪烁等。

```c++
// 03_threads_led/ledcontroller.cpp
// LED灯控制
void LEDController::update_led(void){
    // input your code
}

// LED灯自定义闪烁
// times 代表闪烁的次数
// delay_time_per_blink 代表一次闪烁的时延
void LEDController::blink_faster(int times, uint32_t delay_time_per_blink) {
    //  input your code
}
```

修改`03_threads_led/main.cpp`的 `LED_GPIO`定义以及`void *_led_thread(void *arg)`函数，完成实验要求。具体实现不限于上述内容，可自行实现。
```c++
// 03_threads_led/main.cpp
#define LED_GPIO GPIO12
LEDController led(LED_GPIO);
void *_led_thread(void *arg)
{
    (void) arg;
    // input your code
    return NULL;
}
```

实验要求: 每过2秒，LED灯以每次闪烁100ms的时延闪烁5次。
#### 补充:
1. 引脚模式:`int gpio_init(gpio_t pin, gpio_mode_t mode);` 初始化GPIO引脚(如`GPIO12`)的工作模式，目前支持6种工作模式，具体参考下面内容，本实验只需要通过输出电压来控制LED灯，因此设置为`GPIO_OUT`即可。
```c++
typedef enum {
    GPIO_IN    = GPIO_MODE(0, 0, 0),    /**< input w/o pull R */
    GPIO_IN_PD = GPIO_MODE(0, 2, 0),    /**< input with pull-down */
    GPIO_IN_PU = GPIO_MODE(0, 1, 0),    /**< input with pull-up */
    GPIO_OUT   = GPIO_MODE(1, 0, 0),    /**< push-pull output */
    GPIO_OD    = GPIO_MODE(1, 0, 1),    /**< open-drain w/o pull R */
    GPIO_OD_PU = GPIO_MODE(1, 1, 1)     /**< open-drain with pull-up */
} gpio_mode_t;
```
2. 引脚状态设置。RIOT控制引脚电压有两种方式。

(1) 方法一

`void gpio_write(gpio_t pin, int value);` value 为0时，引脚为低电压，value为1时, 引脚为高电压。

(2) 方法二

`void gpio_set(gpio_t pin);` 将引脚pin设置为高电压。

`void gpio_clear(gpio_t pin);` 将引脚pin设置为低电压。

### IMU惯性传感器使用及LED展示