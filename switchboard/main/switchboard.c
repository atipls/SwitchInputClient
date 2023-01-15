#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "nvs.h"
#include "nvs_flash.h"

#define INCLUDED_FROM_SWITCHBOARD
#include "blobs.c"

#define LED_GPIO 2
#define PIN_SEL (1ULL << LED_GPIO)

enum {
    DPAD_UP = 0,
    DPAD_UP_RIGHT,
    DPAD_RIGHT,
    DPAD_DOWN_RIGHT,
    DPAD_DOWN,
    DPAD_DOWN_LEFT,
    DPAD_LEFT,
    DPAD_UP_LEFT,
    DPAD_CENTER,
};

enum {
    DPAD_SET_UP = 1 << 0,
    DPAD_SET_DOWN = 1 << 1,
    DPAD_SET_LEFT = 1 << 2,
    DPAD_SET_RIGHT = 1 << 3,
};

static struct {
    uint8_t buttons1;// (Right)  Y, X, B, A, SR, SL, R, ZR
    uint8_t buttons2;// (Shared)  -, +, Rs, Ls, H, Cap, --, Charging Grip
    uint8_t buttons3;// (Left)  D, U, R, L, SR, SL, L, ZL

    uint8_t lx;
    uint8_t ly;

    uint8_t cx;
    uint8_t cy;
} xReport;

SemaphoreHandle_t xSemaphore;

bool connected = false;
int paired = 0;

TaskHandle_t appUartTaskHandle = NULL;
TaskHandle_t appSendTaskHandle = NULL;

static esp_hidd_app_param_t app_param;
static esp_hidd_qos_param_t qos_param;

uint8_t timer = 0;

uart_config_t uart_config;
QueueHandle_t uart_queue;

#define BUF_SIZE (8192)

static uint8_t *uart_data;

static uint8_t uartUpdateCrc(uint8_t previous_crc, uint8_t new_data) {
    uint8_t current = previous_crc ^ new_data;

    for (uint8_t i = 0; i < 8; i++) {
        if ((current & 0x80) != 0) {
            current <<= 1;
            current ^= 0x07;
        } else {
            current <<= 1;
        }
    }
    return current;
}

uint8_t appGetUartStatus(uint8_t data) {
    switch (uart_data[2]) {
        case DPAD_UP: return DPAD_SET_UP;
        case DPAD_UP_RIGHT: return DPAD_SET_UP | DPAD_SET_RIGHT;
        case DPAD_RIGHT: return DPAD_SET_RIGHT;
        case DPAD_DOWN_RIGHT: return DPAD_SET_DOWN | DPAD_SET_RIGHT;
        case DPAD_DOWN: return DPAD_SET_DOWN;
        case DPAD_DOWN_LEFT: return DPAD_SET_DOWN | DPAD_SET_LEFT;
        case DPAD_LEFT: return DPAD_SET_LEFT;
        case DPAD_UP_LEFT: return DPAD_SET_UP | DPAD_SET_LEFT;
        case DPAD_CENTER:
        default: return 0;
    }
}

void appUARTInit() {
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2,
                                        10, &uart_queue, 0));

    uart_data = (uint8_t *) malloc(BUF_SIZE);
}

static void appUARTTask() {
    while (true) {
        int length = uart_read_bytes(UART_NUM_0, uart_data, BUF_SIZE, 0);
        if (length < 9) {
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        uint8_t current_crc = 0;
        for (uint8_t i = 0; i < 8; i++)
            current_crc = uartUpdateCrc(current_crc, uart_data[i]);

        if (current_crc != uart_data[8]) {
            ESP_LOGI("CRC Error",
                     "Packet specified CRC 0x%02x but calculated CRC was 0x%02x",
                     uart_data[8], current_crc);
            ESP_LOGI("CRC Error",
                     "Packet data: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                     uart_data[0], uart_data[1], uart_data[2], uart_data[3],
                     uart_data[4], uart_data[5], uart_data[6], uart_data[7],
                     uart_data[8]);
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }
        uint8_t dpad_status = appGetUartStatus(uart_data[2]);

        // This is not a joycon, so no SL/SR.
        uart_data[0] &= ~(1 << 7);// Clear SL
        uart_data[0] &= ~(1 << 6);// Clear SR

        xReport.buttons1 = (((uart_data[1] >> 0) & 1) << 0) |// Y
                           (((uart_data[1] >> 3) & 1) << 1) |// X
                           (((uart_data[1] >> 1) & 1) << 2) |// B
                           (((uart_data[1] >> 2) & 1) << 3) |// A
                           (((uart_data[0] >> 7) & 1) << 4) |// SR
                           (((uart_data[0] >> 6) & 1) << 5) |// SL
                           (((uart_data[1] >> 5) & 1) << 6) |// R
                           (((uart_data[1] >> 7) & 1) << 7); // ZR

        xReport.buttons2 = (((uart_data[0] >> 0) & 1) << 0) |// -/Select
                           (((uart_data[0] >> 1) & 1) << 1) |// +/Start
                           (((uart_data[0] >> 3) & 1) << 2) |// R Stick Click
                           (((uart_data[0] >> 2) & 1) << 3) |// L Stick Click
                           (((uart_data[0] >> 4) & 1) << 4) |// Home
                           (((uart_data[0] >> 5) & 1) << 5); // Capture

        xReport.buttons3 = (((dpad_status & DPAD_SET_DOWN) ? 0 : 1) << 0) | // Down
                           (((dpad_status & DPAD_SET_UP) ? 0 : 1) << 1) |   // Up
                           (((dpad_status & DPAD_SET_RIGHT) ? 0 : 1) << 2) |// Right
                           (((dpad_status & DPAD_SET_LEFT) ? 0 : 1) << 3) | // Left
                           (((uart_data[0] >> 7) & 1) << 4) |               // SR
                           (((uart_data[0] >> 6) & 1) << 5) |               // SL
                           (((uart_data[1] >> 4) & 1) << 6) |               // L
                           (((uart_data[1] >> 6) & 1) << 7);                // ZL


        xReport.lx = uart_data[3];
        xReport.ly = 255 - uart_data[4];

        xReport.cx = uart_data[5];
        xReport.cy = 255 - uart_data[6];
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

static uint8_t report30[48] = {[0] = 0x00, [1] = 0x8E, [11] = 0x80};

void appSENDTask() {
    ESP_LOGI("SENDER", "Sending hid reports on core %d\n", xPortGetCoreID());

    while (true) {
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

        report30[0] = timer;

        report30[2] = xReport.buttons1;
        report30[3] = xReport.buttons2;
        report30[4] = xReport.buttons3;

        report30[5] = (xReport.lx << 4) & 0xF0;
        report30[6] = (xReport.lx & 0xF0) >> 4;
        report30[7] = xReport.ly;

        report30[8] = (xReport.cx << 4) & 0xF0;
        report30[9] = (xReport.cx & 0xF0) >> 4;
        report30[10] = xReport.cy;

        xSemaphoreGive(xSemaphore);

        timer += 1;
        if (timer == 255)
            timer = 0;

        if (paired || connected) {
            esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30,
                                          sizeof(report30), report30);
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

esp_err_t set_bt_address() {
    nvs_handle nvs_handle;
    uint8_t bt_addr[8];

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    size_t addr_size = 0;
    err = nvs_get_blob(nvs_handle, "mac_addr", NULL, &addr_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
        return err;

    if (addr_size > 0) {
        err = nvs_get_blob(nvs_handle, "mac_addr", bt_addr, &addr_size);
    } else {
        for (uint8_t i = 0; i < 8; i++)
            bt_addr[i] = random() % 255;
        size_t addr_size = sizeof(bt_addr);
        err = nvs_set_blob(nvs_handle, "mac_addr", bt_addr, addr_size);
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    esp_base_mac_addr_set(bt_addr);

    // put mac addr in switch pairing packet
    for (uint8_t z = 0; z < 6; z++)
        reply02[z + 19] = bt_addr[z];

    return ESP_OK;
}

#define SPP_TAG "tag"
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event,
                          esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
            esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
            break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
            break;
        case ESP_BT_GAP_RMT_SRVCS_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
            ESP_LOGI(SPP_TAG, "%d", param->rmt_srvcs.num_uuids);
            break;
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
            break;
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(SPP_TAG, "authentication success: %s",
                         param->auth_cmpl.device_name);
                esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } else {
                ESP_LOGE(SPP_TAG, "authentication failed, status:%d",
                         param->auth_cmpl.stat);
            }
            break;
        }

        default:
            break;
    }
}

void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
    static const char *TAG = "esp_bt_hidd_cb";

    switch (event) {
        case ESP_HIDD_INIT_EVT:
            if (param->init.status == ESP_HIDD_SUCCESS) {
                ESP_LOGI(TAG, "setting hid parameters");
                esp_bt_hid_device_register_app(&app_param, &qos_param, &qos_param);
            } else {
                ESP_LOGE(TAG, "init hidd failed!");
            }
            break;
        case ESP_HIDD_DEINIT_EVT:
            break;
        case ESP_HIDD_REGISTER_APP_EVT:
            if (param->register_app.status == ESP_HIDD_SUCCESS) {
                ESP_LOGI(TAG, "setting hid parameters success!");
                ESP_LOGI(TAG, "setting to connectable, discoverable");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                if (param->register_app.in_use) {
                    ESP_LOGI(TAG, "start virtual cable plug!");
                    esp_bt_hid_device_connect(param->register_app.bd_addr);
                }
            } else {
                ESP_LOGE(TAG, "setting hid parameters failed!");
            }
            break;
        case ESP_HIDD_UNREGISTER_APP_EVT:
            if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
                ESP_LOGI(TAG, "unregister app success!");
            } else {
                ESP_LOGE(TAG, "unregister app failed!");
            }
            break;
        case ESP_HIDD_OPEN_EVT:
            if (param->open.status == ESP_HIDD_SUCCESS) {
                switch (param->open.conn_status) {
                    case ESP_HIDD_CONN_STATE_CONNECTING:
                        ESP_LOGI(TAG, "connecting...");
                        break;
                    case ESP_HIDD_CONN_STATE_CONNECTED:
                        ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x",
                                 param->open.bd_addr[0], param->open.bd_addr[1],
                                 param->open.bd_addr[2], param->open.bd_addr[3],
                                 param->open.bd_addr[4], param->open.bd_addr[5]);

                        ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE,
                                                 ESP_BT_NON_DISCOVERABLE);

                        xSemaphoreTake(xSemaphore, portMAX_DELAY);
                        connected = true;
                        xSemaphoreGive(xSemaphore);

                        if (appSendTaskHandle != NULL) {
                            vTaskDelete(appSendTaskHandle);
                            appSendTaskHandle = NULL;
                        }

                        xTaskCreatePinnedToCore(appSENDTask, "SEND Task", 4096, NULL, 2,
                                                &appSendTaskHandle, 0);
                        break;
                    default:
                        break;
                }
            } else {
                ESP_LOGE(TAG, "open failed!");
            }
            break;
        case ESP_HIDD_CLOSE_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
            if (param->close.status == ESP_HIDD_SUCCESS) {
                if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                    ESP_LOGI(TAG, "disconnecting...");
                } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                    ESP_LOGI(TAG, "disconnected!");
                    ESP_LOGI(TAG, "making self discoverable and connectable again.");
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                             ESP_BT_GENERAL_DISCOVERABLE);

                    xSemaphoreTake(xSemaphore, portMAX_DELAY);
                    connected = false;
                    paired = 0;
                    xSemaphoreGive(xSemaphore);
                } else {
                    ESP_LOGE(TAG, "unknown connection status");
                }
            } else {
                ESP_LOGE(TAG, "close failed!");
            }
            break;
        case ESP_HIDD_SEND_REPORT_EVT: break;
        case ESP_HIDD_REPORT_ERR_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
            break;
        case ESP_HIDD_GET_REPORT_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d",
                     param->get_report.report_id, param->get_report.report_type,
                     param->get_report.buffer_size);
            break;
        case ESP_HIDD_SET_REPORT_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
            break;
        case ESP_HIDD_SET_PROTOCOL_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
            if (param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE) {
                ESP_LOGI(TAG, "  - boot protocol");
            } else if (param->set_protocol.protocol_mode == ESP_HIDD_REPORT_MODE) {
                ESP_LOGI(TAG, "  - report protocol");
            }
            break;
        case ESP_HIDD_INTR_DATA_EVT:
            // ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
            // ESP_LOG_BUFFER_HEX(TAG, param->intr_data.data, param->intr_data.len);

            if (param->intr_data.data[9] == 2)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply02), reply02);
            if (param->intr_data.data[9] == 8)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply08), reply08);
            if (param->intr_data.data[9] == 16 && param->intr_data.data[10] == 0 &&
                param->intr_data.data[11] == 96)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(spi_reply_address_0),
                                              spi_reply_address_0);
            if (param->intr_data.data[9] == 16 && param->intr_data.data[10] == 80 &&
                param->intr_data.data[11] == 96)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(spi_reply_address_0x50),
                                              spi_reply_address_0x50);
            if (param->intr_data.data[9] == 3)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply03), reply03);
            if (param->intr_data.data[9] == 4)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply04), reply04);
            if (param->intr_data.data[9] == 16 && param->intr_data.data[10] == 128 &&
                param->intr_data.data[11] == 96)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(spi_reply_address_0x80),
                                              spi_reply_address_0x80);
            if (param->intr_data.data[9] == 16 && param->intr_data.data[10] == 152 &&
                param->intr_data.data[11] == 96)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(spi_reply_address_0x98),
                                              spi_reply_address_0x98);
            if (param->intr_data.data[9] == 16 && param->intr_data.data[10] == 16 &&
                param->intr_data.data[11] == 128)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(spi_reply_address_0x10),
                                              spi_reply_address_0x10);
            if (param->intr_data.data[9] == 16 && param->intr_data.data[10] == 61 &&
                param->intr_data.data[11] == 96)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(spi_reply_address_0x3d),
                                              spi_reply_address_0x3d);
            if (param->intr_data.data[9] == 16 && param->intr_data.data[10] == 32 &&
                param->intr_data.data[11] == 96)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(spi_reply_address_0x20),
                                              spi_reply_address_0x20);
            if (param->intr_data.data[9] == 64)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply4001), reply4001);
            if (param->intr_data.data[9] == 72)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply4801), reply4801);
            if (param->intr_data.data[9] == 34)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply3401), reply3401);
            if (param->intr_data.data[9] == 48)
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply3001), reply3001);

            if (param->intr_data.data[9] == 33 && param->intr_data.data[10] == 33) {
                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                              sizeof(reply3333), reply3333);
                paired = 1;
            }

            break;
        case ESP_HIDD_VC_UNPLUG_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
            if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
                if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                    connected = false;
                    paired = false;
                    ESP_LOGI(TAG, "disconnected!");

                    ESP_LOGI(TAG, "making self discoverable and connectable again.");
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                             ESP_BT_GENERAL_DISCOVERABLE);
                } else {
                    ESP_LOGE(TAG, "unknown connection status");
                }
            } else {
                ESP_LOGE(TAG, "close failed!");
            }
            break;
        default:
            break;
    }
}

void app_main() {
    appUARTInit();

    xSemaphore = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(appUARTTask, "UART Task", 2048, NULL, 1,
                            &appUartTaskHandle, 1);

    esp_err_t ret;

    static esp_bt_cod_t dclass;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    app_param.name = "Wireless Gamepad";
    app_param.description = "Gamepad";
    app_param.provider = "Nintendo";
    app_param.subclass = 0x8;
    app_param.desc_list = hid_descriptor;
    app_param.desc_list_len = hid_descriptor_len;

    memset(&qos_param, 0, sizeof(esp_hidd_qos_param_t));

    dclass.minor = 2;
    dclass.major = 5;
    dclass.service = 1;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));

    esp_bt_dev_set_device_name("Pro Controller");

    esp_bt_gap_set_cod(dclass, ESP_BT_SET_COD_ALL);
    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);
    esp_bt_hid_device_init();
}