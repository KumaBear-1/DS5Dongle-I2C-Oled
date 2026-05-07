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

// --- 新增：OLED 屏幕相关的头文件和定义 ---
#include "hardware/i2c.h" // Pico I2C 库
#include "SSD1306.hpp" 
#include "GFX.hpp"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

class Display : public SSD1306, public GFX {
public:
    Display() : SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT), GFX(SCREEN_WIDTH, SCREEN_HEIGHT) {}
};

Display display; // 实例化对象

// Pico SDK specifically for waiting on conditions
#include "pico/critical_section.h"

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

    // TODO: Refactor for better code reuse
    if (get_config().polling_rate_mode != 2) {
        if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    // Local buffer to hold the report data while we prepare it to send. 
    uint8_t safe_report;


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
    if (channel == INTERRUPT && data == 0x31) {
        if ((data & 1) != (interrupt_in_data & 1)) {
            set_headset(data & 1);
        }<websource>source_group_web_1</websource>

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
        printf("[HID] Receive 0xf6 setting config, funcid:0x%02X\n",buffer);
        pico_cmd_set(report_id,buffer,bufsize);
        return;
    }

    // INTERRUPT OUT
    if (report_id == 0) {
        switch (buffer) {
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
    if (report_id == 0x80 ||
        // DSE: Write Profile Block
        report_id == 0x60 ||
        report_id == 0x62 ||
        report_id == 0x61) {
        set_feature_data(report_id,const_cast<uint8_t *>(buffer),bufsize);
        return;
    }
}<websource>source_group_web_2</websource>

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
        // 当崩溃重启以后，闪三下灯
        for (int i = 0;i < 6;i++) {
            if (i % 2 == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            }else {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            }
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
  
    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);

    config_load();

    bt_init();
    bt_register_data_callback(on_bt_data);

    audio_init();

    watchdog_enable(1000, true);

    // --- 新增：OLED 屏幕初始化代码 ---
    // 1. 初始化 I2C 接口 (使用 i2c0)
    i2c_init(i2c0, 400 * 1000); // 设置波特率为 400kHz
    
    // 2. 配置 I2C 引脚 (根据你的实际接线修改 GPIO 编号)
    // 这里假设 SDA 接在 GP4, SCL 接在 GP5
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_pull_up(15);
    gpio_pull_up(14);

    // 3. 初始化并启动显示屏
    if (!display.begin(i2c0, 0x3C)) {
        printf("OLED 屏幕初始化失败！\n");
        // 如果初始化失败，可以选择停止程序或继续运行
        while(true) {
            tight_loop_contents();
        }
    }
    // --- 初始化结束 ---

    while (1) {
        watchdog_update();
        cyw43_arch_poll();
        tud_task();
        audio_loop();
        interrupt_loop();

        // --- 新增：OLED 屏幕显示代码 ---
        display.clear();             // 1. 清空缓冲区
        display.setTextColor(WHITE); // 2. 设置文字颜色为白色
        display.setCursor(0, 0);     // 3. 设置光标位置到左上角
        display.print("12345");      // 4. 写入字符串
        display.update();            // 5. 刷新屏幕，让内容显示出来
        // --- 显示结束 ---
    }
}
