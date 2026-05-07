#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// 定义引脚 (GP14=SCL, GP15=SDA)
#define I2C_PORT i2c1
#define PIN_SDA 15
#define PIN_SCL 14

int main() {
    // 1. 初始化串口 (用于在电脑上显示结果)
    stdio_init_all();
    
    // 2. 初始化 I2C
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz
    
    // 3. 设置 GPIO 功能为 I2C
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    
    // 4. 开启内部上拉电阻 (非常重要，防止信号悬空)
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    printf(">>> 开始扫描 I2C 总线 (GP14/15)... \n");

    int found_count = 0;

    // 5. 遍历所有可能的 I2C 地址 (0x00 - 0x7F)
    for (uint8_t addr = 0; addr < 128; ++addr) {
        uint8_t rxdata;
        // 尝试读取 1 个字节
        int ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);

        // 如果返回值 >= 0，说明有设备响应了这个地址
        if (ret >= 0) {
            printf("   [成功] 发现设备！地址: 0x%02X\n", addr);
            found_count++;
        }
    }

    if (found_count == 0) {
        printf("   [失败] 未找到任何 I2C 设备。\n");
        printf("   请检查：\n");
        printf("   1. OLED 的 VCC 是否接到了 3V3 或 VSYS？\n");
        printf("   2. OLED 的 GND 是否接到了 GND？(必须共地)\n");
        printf("   3. SDA/SCL 线是否松动？\n");
    } else {
        printf(">>> 扫描结束，共发现 %d 个设备。\n", found_count);
    }

    while (1) {
        tight_loop_contents();
    }
}
