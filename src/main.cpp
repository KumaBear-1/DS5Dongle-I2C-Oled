int main() {
    // 1. 系统时钟与电源设置
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(320000, true);

    // 2. 板级初始化 (必须在调用任何外设驱动前完成)
    board_init();
    
    // 3. USB 初始化
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    tud_disconnect();
    board_init_after_tusb();

    // 4. WiFi/蓝牙芯片初始化
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    // 5. 看门狗状态检查
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        for (int i = 0; i < 6; i++) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, i % 2 == 0);
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
  
    // 6. 关键区域与配置加载
    critical_section_init(&report_cs);
    config_load();
    bt_init();
    bt_register_data_callback(on_bt_data);
    audio_init();
    watchdog_enable(1000, true);

    // --- OLED 屏幕初始化 (放在这里，硬件已就绪) ---
    // 假设 display 是全局对象，且构造函数已分配内存
    // 注意：如果你的 SSD1306 库没有 begin 函数，请去掉这个 if 判断
    // 如果必须初始化 I2C，确保 GPIO 定义正确
    i2c_init(i2c0, 400 * 1000); 
    gpio_set_function(15, GPIO_FUNC_I2C); // SDA
    gpio_set_function(14, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(15);
    gpio_pull_up(14);

    // 尝试初始化屏幕 (根据库的实际 API 调整)
    // 如果报错说没有 begin，直接删掉这一行，有些库构造即初始化
    if (!display.begin(i2c0, 0x3C)) { 
        printf("OLED init failed!\n");
    }
    
    // 开机画面测试
    display.clear(SSD1306::colors::BLACK);
    display.drawString(0, 0, "Hello", SSD1306::colors::WHITE);
    display.show(); 

    // 7. 主循环
    while (1) {
        watchdog_update();
        cyw43_arch_poll();
        tud_task();
        audio_loop();
        interrupt_loop();

        // --- OLED 刷新逻辑 ---
        // 注意：频繁清屏和重绘可能会导致闪烁，建议只更新变化的部分
        display.clear(SSD1306::colors::BLACK);
        display.drawString(0, 0, "DS5 Dongle", SSD1306::colors::WHITE);
        display.drawString(0, 10, "Running...", SSD1306::colors::WHITE);
        display.show();
    }
}
