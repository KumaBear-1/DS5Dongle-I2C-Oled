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
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/critical_section.h"

#define OLED_ADDR 0x3C

// --- 修改点：使用正确的枚举值 size::W128xH64 ---
class Display : public SSD1306, public GFX {
public:
    // 父类构造函数需要：(地址, 尺寸枚举, i2c指针)
    Display() : SSD1306(OLED_ADDR, size::W128xH64, i2c0), GFX(OLED_ADDR, size::W128xH64, i2c0) {}
};

Display display; // 实例化对象

int reportSeqCounter = 0;
uint8_t packetCounter = 0;

uint8_t interrupt_in_data = {
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

    if (get_config().polling_rate_mode != 2) {
        if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    uint8_t safe_report;

    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, 63);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, 63)) {
            printf("[USBHID] tud_hid_report error\n");
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

void on_bt_data(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    if (channel == INTERRUPT && data == 0x31) {
        if ((data & 1) != (interrupt_in_data & 1)) {
            set_headset(data & 1);
        }

        if (get_config().polling_rate_mode != 2) {
            memcpy(interrupt_in_data, data + 3, 63);
            return;
        }

        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, 63);
        report_dirty = true;
        critical_section_exit(&report_cs);
    }
}

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

    if (report_id == 0) {
        switch (buffer[0]) {
            case 0x02: {
                uint8_t outputData;
                outputData = 0x31;
                outputData = reportSeqCounter << 4;
                if (++reportSeqCounter == 256) {
                    reportSeqCounter = 0;
                }
                outputData = 0x10;
                memcpy(outputData + 3, buffer + 1, bufsize - 1);
                bt_write(outputData, sizeof(outputData));
                break;
            }
        }
    }
    if (report_id == 0x80 || report_id == 0x60 || report_id == 0x62 || report_id == 0x61) {
        set_feature_data(report_id,const_cast<uint8_t *>(buffer),bufsize);
        return;
    }
}

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(320000, true);

    board_init();
    
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    tud_disconnect();
    board_init_after_tusb();

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        for (int i = 0; i < 6; i++) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, i % 2 == 0);
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
  
    critical_section_init(&report_cs);
    config_load();
    bt_init();
    bt_register_data_callback(on_bt_data);
    audio_init();
    watchdog_enable(1000, true);

    // --- OLED 初始化 GPIO ---
    i2c_init(i2c0, 400 * 1000); 
    gpio_set_function(15, GPIO_FUNC_I2C); // SDA
    gpio_set_function(14, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(15);
    gpio_pull_up(14);

    // --- 测试显示 ---
    // 1. 清屏 (直接使用 colors::BLACK，因为 Display 继承了该类)
    display.clear(colors::BLACK);
    
    // 2. 绘制文字 (假设 GFX 类有 drawString)
    // 如果 GFX 没有 drawString，可能需要用 print 或其他方法
    display.drawString(0, 0, "Hello", colors::WHITE);
    
    // 3. 刷新屏幕 (使用 display() 函数，不是 show())
    display.display(); 

    while (1) {
        watchdog_update();
        cyw43_arch_poll();
        tud_task();
        audio_loop();
        interrupt_loop();

        // --- OLED 循环显示 ---
        display.clear(colors::BLACK);
        display.drawString(0, 0, "DS5 Dongle", colors::WHITE);
        display.drawString(0, 10, "Running...", colors::WHITE);
        display.display(); // 再次调用 display() 刷新
    }
}
