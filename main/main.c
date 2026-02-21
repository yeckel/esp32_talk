/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "bt_app_core.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_ag_api.h"
#include "bt_app_hf.h"
#include "gpio_pcm_config.h"
#include "esp_console.h"
#include "app_hf_msg_set.h"
#include "argtable3/argtable3.h"
#include "walkie_talkie.h"

#define BT_HF_AG_TAG    "WT_MAIN"

/* event for handler "hf_ag_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* Helper function to parse MAC address string */
static bool str2bda(const char *str, esp_bd_addr_t bda)
{
    int tmp[6];
    if (sscanf(str, "%x:%x:%x:%x:%x:%x", 
               &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        bda[i] = (uint8_t)tmp[i];
    }
    return true;
}

/* GAP callback function for device discovery and pairing */
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        // Device discovered during inquiry
        ESP_LOGI(BT_HF_AG_TAG, "Device found: %02x:%02x:%02x:%02x:%02x:%02x",
                 param->disc_res.bda[0], param->disc_res.bda[1],
                 param->disc_res.bda[2], param->disc_res.bda[3],
                 param->disc_res.bda[4], param->disc_res.bda[5]);
        
        // Print device name and other properties if available
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                ESP_LOGI(BT_HF_AG_TAG, "  Name: %s", (char *)param->disc_res.prop[i].val);
            } else if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_COD) {
                uint32_t cod = *(uint32_t *)(param->disc_res.prop[i].val);
                ESP_LOGI(BT_HF_AG_TAG, "  Class of Device: 0x%"PRIx32, cod);
            } else if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_RSSI) {
                int8_t rssi = *(int8_t *)(param->disc_res.prop[i].val);
                ESP_LOGI(BT_HF_AG_TAG, "  RSSI: %d", rssi);
            }
        }
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(BT_HF_AG_TAG, "Device discovery stopped.");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(BT_HF_AG_TAG, "Device discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT: {
        ESP_LOGI(BT_HF_AG_TAG, "Remote services discovered for: %02x:%02x:%02x:%02x:%02x:%02x",
                 param->rmt_srvcs.bda[0], param->rmt_srvcs.bda[1],
                 param->rmt_srvcs.bda[2], param->rmt_srvcs.bda[3],
                 param->rmt_srvcs.bda[4], param->rmt_srvcs.bda[5]);
        break;
    }
    case ESP_BT_GAP_RMT_SRVC_REC_EVT: {
        ESP_LOGI(BT_HF_AG_TAG, "Remote service record received");
        break;
    }
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_HF_AG_TAG, "✓ Authentication SUCCESS: %s", param->auth_cmpl.device_name);
            ESP_LOGI(BT_HF_AG_TAG, "  Device: %02x:%02x:%02x:%02x:%02x:%02x",
                     param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                     param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                     param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
            ESP_LOGI(BT_HF_AG_TAG, "  Device is now paired! You can now connect with: con <mac>");
        } else {
            ESP_LOGE(BT_HF_AG_TAG, "✗ Authentication FAILED, status: %d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(BT_HF_AG_TAG, "PIN Request from device");
        ESP_LOGI(BT_HF_AG_TAG, "Sending PIN: 0000");
        esp_bt_pin_code_t pin_code;
        pin_code[0] = '0';
        pin_code[1] = '0';
        pin_code[2] = '0';
        pin_code[3] = '0';
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT: {
        ESP_LOGI(BT_HF_AG_TAG, "SSP Confirmation Request - Passkey: %"PRIu32, param->cfm_req.num_val);
        ESP_LOGI(BT_HF_AG_TAG, "Auto-confirming pairing...");
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    }
    case ESP_BT_GAP_KEY_NOTIF_EVT: {
        ESP_LOGI(BT_HF_AG_TAG, "SSP Passkey Notification: %"PRIu32, param->key_notif.passkey);
        break;
    }
    case ESP_BT_GAP_KEY_REQ_EVT: {
        ESP_LOGI(BT_HF_AG_TAG, "SSP Passkey Request - Please enter passkey");
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT: {
        ESP_LOGI(BT_HF_AG_TAG, "Mode changed - mode: %d", param->mode_chg.mode);
        break;
    }
    default: {
        ESP_LOGI(BT_HF_AG_TAG, "GAP event: %d", event);
        break;
    }
    }
}

/* handler for bluetooth stack enabled events */
static void bt_hf_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_HF_TAG, "%s evt %d", __func__, event);
    switch (event)
    {
        case BT_APP_EVT_STACK_UP:
        {
            /* set up device name */
            char *dev_name = "ESP_HFP_AG";
            esp_bt_gap_set_device_name(dev_name);

            /* Register GAP callback for device discovery and pairing */
            esp_bt_gap_register_callback(esp_bt_gap_cb);

            /* Set up Secure Simple Pairing (SSP) - NoInputNoOutput for auto-pairing */
            esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
            esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));

            /* Enable SSP */
            esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
            esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

            esp_hf_ag_register_callback(bt_app_hf_cb);

            // init and register for HFP_AG functions
            esp_hf_ag_init();

            /*
            * Set default parameters for Legacy Pairing
            * Use variable pin, input pin code when pairing
            */
            esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '0';
            pin_code[1] = '0';
            pin_code[2] = '0';
            pin_code[3] = '0';
            esp_bt_gap_set_pin(pin_type, 4, pin_code);

            /* set discoverable and connectable mode, wait to be connected */
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;
        }
        default:
            ESP_LOGE(BT_HF_TAG, "%s unhandled evt %d", __func__, event);
            break;
    }
}

void app_main(void)
{
    printf("=== ESP32 Walkie-Talkie starting... ===\n");

    /* Initialize NVS — used for calibration data and WT config */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialise TCP/IP stack and default event loop (required for WiFi/ESP-NOW) */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_HF_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* Initialise ESP32 Walkie-Talkie (Codec2 + ESP-NOW + NVS config) */
    ESP_ERROR_CHECK(walkie_talkie_init());

    /* create application task */
    bt_app_task_start_up();

    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_hf_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

#if CONFIG_BT_HFP_AUDIO_DATA_PATH_PCM
    /* configure the PCM interface and PINs used */
    app_gpio_pcm_io_cfg();
#endif

    /* configure external chip for acoustic echo cancellation */
#if ACOUSTIC_ECHO_CANCELLATION_ENABLE
    app_gpio_aec_io_cfg();
#endif /* ACOUSTIC_ECHO_CANCELLATION_ENABLE */

    /* ---- UART console REPL ----
     * argtable3 (used by esp_console) calls abort() on malloc failure,
     * so guard with a free-heap check.                                */
    esp_console_repl_t *repl = NULL;
    size_t free_before_repl = esp_get_free_heap_size();
    ESP_LOGI(BT_HF_TAG, "Free heap before REPL: %u", (unsigned)free_before_repl);
    if (free_before_repl >= 6144) {
        esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
        repl_config.max_history_len = 4;        /* default 32 → save RAM */
        repl_config.task_stack_size = 3072;      /* default 4096 → save 1 KB */
        repl_config.max_cmdline_length = 128;    /* default 256 → save heap */
        esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
        repl_config.prompt = "hfp_ag>";
        ret = esp_console_new_repl_uart(&uart_config, &repl_config, &repl);
        if (ret == ESP_OK && repl) {
            /* Register commands only if console is initialised */
            register_hfp_ag();
        }
    } else {
        ESP_LOGW(BT_HF_TAG, "Skipping REPL – only %u bytes free (need 6144)",
                 (unsigned)free_before_repl);
        ret = ESP_ERR_NO_MEM;
    }

    printf("\n ===========================================================\n");
    printf(" |   ESP32 Duplex Walkie-Talkie (BT Headset + ESP-NOW)     |\n");
    printf(" |                                                         |\n");
    printf(" |  FIRST-TIME SETUP:                                      |\n");
    printf(" |  1. scan               – find BT headsets               |\n");
    printf(" |  2. pair <MAC>         – pair with headset               |\n");
    printf(" |  3. setmac <MAC>       – save headset MAC                |\n");
    printf(" |  4. setpeer <MAC>      – set remote ESP32 MAC            |\n");
    printf(" |  5. setch <N>          – set ESP-NOW channel (default 1) |\n");
    printf(" |                                                         |\n");
    printf(" |  DAILY USE:                                             |\n");
    printf(" |  con                   – connect to headset              |\n");
    printf(" |  call                  – start audio (auto after con)    |\n");
    printf(" |  bw <0..5|8|9|255>     – codec mode (9=ADPCM,255=raw)    |\n");
    printf(" |  vox <0-500>           – VOX threshold (0=PTT only)       |\n");
    printf(" |  ptt                   – toggle push-to-talk              |\n");
    printf(" |  vu 0 <0-15>           – speaker volume                  |\n");
    printf(" |  vu 1 <0-15>           – mic volume                      |\n");
    printf(" |  status                – show full status                |\n");
    printf(" |  help                  – list all commands               |\n");
    printf(" ===========================================================\n\n");

    // start console REPL
    if (ret == ESP_OK && repl) {
        ESP_ERROR_CHECK(esp_console_start_repl(repl));
    } else {
        ESP_LOGW(BT_HF_TAG, "Console REPL unavailable (%s) – running headless",
                 esp_err_to_name(ret));
    }
}
