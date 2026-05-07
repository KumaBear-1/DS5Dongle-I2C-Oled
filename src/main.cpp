//
// Created by awalol on 2026/3/4.
//

#include <cstdio>
#include "bsp/board_api.h"
#include "bt.h"
#include "utils.h"
#include "resample.h"
#include "audio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "config.h"
#include "cmd.h"
#include "SSD1306.hpp" 
#include "GFX.hpp"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

class Display : public SSD1306, public GFX {
public:
    Display() : SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT), GFX(SCREEN_WIDTH, SCREEN_HEIGHT) {}
};

Display display; // 实例化对象

// Pico SDK speciifically for waiting on conditions
#include "pico/critical_section.h"

int reportSeqCounter = 0;
uint8_t packetCounter = 0;

uint8_t interrupt_in_data[63] = {
    0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7,
    0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41,
    0x01, 0x00, 0x0e, 0x00, 0xef, 0xff, 0x03, 0x03,
    0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00,
    0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00,
    0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b
};

critical_section_t report_cs;
volatile bool report_dirty = false;

void interrupt_loop() {
    if (!tud_hid_ready()) return;

    // TODO: Refactor for better code reuse
    if (get_config().polling_rate_mode != 2) {
        if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    // Local buffer to hold the report data while we prepare it to send. 
    uint8_t safe_report[63];


    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, 63);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, 63)) {
            printf("[USBHID] tud_hid_report error\n");
            
            // If the report failed to queue, restore the dirty flag 
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

void on_bt_data(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // printf("[Main] BT data callback: channel=%u len=%u\n", channel, len);
    if (channel == INTERRUPT && data[1] == 0x31) {
        if ((data[56] & 1) != (interrupt_in_data[53] & 1)) {
            set_headset(data[56] & 1);
        }

        if (get_config().polling_rate_mode != 2) {
            memcpy(interrupt_in_data, data + 3, 63);
            return;
        }

        // We add the critical section here to avoid any race conditions when writing to the interrupt_in_data buffer,
        // which is shared between the main loop and this callback. 
        // The critical section ensures that only one thread can access the buffer at a time, 
        // preventing data corruption and ensuring thread safety.   
        // We also set the report_dirty flag to true to indicate that new data is available
        //  and needs to be sent in the next interrupt report.
        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, 63);
        report_dirty = true;
        critical_section_exit(&report_cs);
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    if (is_pico_cmd(report_id)) {
        return pico_cmd_get(report_id,buffer,reqlen);
    }

    std::vector<uint8_t> feature_data = get_feature_data(report_id, reqlen);
    if (!feature_data.empty()) {
        memcpy(buffer, feature_data.data() + 1, feature_data.size() - 1);
    }

    return feature_data.empty() ? 0 : feature_data.size() - 1;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;

    if (is_pico_cmd(report_id)) {
        printf("[HID] Receive 0xf6 setting config, funcid:0x%02X\n",buffer[0]);
        pico_cmd_set(report_id,buffer,bufsize);
        return;
    }

    // INTERRUPT OUT
    if (report_id == 0) {
        switch (buffer[0]) {
            case 0x02: {
                uint8_t outputData[78];
                outputData[0] = 0x31;
                outputData[1] = reportSeqCounter << 4;
                if (++reportSeqCounter == 256) {
                    reportSeqCounter = 0;
                }
                outputData[2] = 0x10;
                memcpy(outputData + 3, buffer + 1, bufsize - 1);
                bt_write(outputData, sizeof(outputData));
                break;
            }
        }
    }
    if (report_id == 0x80 ||
        // DSE: Write Profile Block
        report_id == 0x60 ||
        report_id == 0x62 ||
        report_id == 0x61) {
        set_feature_data(report_id,const_cast<uint8_t *>(buffer),bufsize);
        return;
    }
}

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
