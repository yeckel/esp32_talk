/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_ag_api.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "time.h"
#include "sys/time.h"
#include "sdkconfig.h"
#include "bt_app_core.h"
#include "bt_app_hf.h"
#include "osi/allocator.h"
#include "walkie_talkie.h"

const char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< SERVICE LEVEL CONNECTION STATE CONTROL */
    "AUDIO_STATE_EVT",                   /*!< AUDIO CONNECTION STATE CONTROL */
    "VR_STATE_CHANGE_EVT",               /*!< VOICE RECOGNITION CHANGE */
    "VOLUME_CONTROL_EVT",                /*!< AUDIO VOLUME CONTROL */
    "UNKNOW_AT_CMD",                     /*!< UNKNOW AT COMMAND RECIEVED */
    "IND_UPDATE",                        /*!< INDICATION UPDATE */
    "CIND_RESPONSE_EVT",                 /*!< CALL & DEVICE INDICATION */
    "COPS_RESPONSE_EVT",                 /*!< CURRENT OPERATOR EVENT */
    "CLCC_RESPONSE_EVT",                 /*!< LIST OF CURRENT CALL EVENT */
    "CNUM_RESPONSE_EVT",                 /*!< SUBSCRIBER INFORTMATION OF CALL EVENT */
    "DTMF_RESPONSE_EVT",                 /*!< DTMF TRANSFER EVT */
    "NREC_RESPONSE_EVT",                 /*!< NREC RESPONSE EVT */
    "ANSWER_INCOMING_EVT",               /*!< ANSWER INCOMING EVT */
    "REJECT_INCOMING_EVT",               /*!< AREJECT INCOMING EVT */
    "DIAL_EVT",                          /*!< DIAL INCOMING EVT */
    "WBS_EVT",                           /*!< CURRENT CODEC EVT */
    "BCS_EVT",                           /*!< CODEC NEGO EVT */
    "PKT_STAT_EVT",                      /*!< REQUEST PACKET STATUS EVT */
};

//esp_hf_connection_state_t
const char *c_connection_state_str[] = {
    "DISCONNECTED",
    "CONNECTING",
    "CONNECTED",
    "SLC_CONNECTED",
    "DISCONNECTING",
};

// esp_hf_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

/// esp_hf_vr_state_t
const char *c_vr_state_str[] = {
    "Disabled",
    "Enabled",
};

// esp_hf_nrec_t
const char *c_nrec_status_str[] = {
    "NREC DISABLE",
    "NREC ABLE",
};

// esp_hf_control_target_t
const char *c_volume_control_target_str[] = {
    "SPEAKER",
    "MICROPHONE",
};

// esp_hf_subscriber_service_type_t
char *c_operator_name_str[] = {
    "China Mobile",
    "China Unicom",
    "China Telecom",
};

// esp_hf_subscriber_service_type_t
char *c_subscriber_service_type_str[] = {
    "UNKNOWN",
    "VOICE",
    "FAX",
};

// esp_hf_nego_codec_status_t
const char *c_codec_mode_str[] = {
    "CVSD Only",
    "Use CVSD",
    "Use MSBC",
};

#if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI

/* 7500 µs = 12 BT slots, one PCM block at 8 kHz CVSD = 120 bytes         */
#define PCM_BLOCK_DURATION_US    7500ULL
#define WBS_PCM_INPUT_DATA_SIZE  240   /* 16kHz * 7.5ms * 2 bytes/sample  */
#define PCM_INPUT_DATA_SIZE      120   /*  8kHz * 7.5ms * 2 bytes/sample  */
/* Timer fires every 4 ms; we accumulate duration and signal HFP when      */
/* at least one full block is ready.                                        */
#define PCM_GENERATOR_TICK_US    4000ULL

static esp_hf_audio_state_t s_audio_code;
static esp_timer_handle_t   s_outgoing_timer = NULL;
static uint64_t             s_last_enter_time = 0;

/* ---- Outgoing callback: fill HFP speaker from decoded ESP-NOW audio ---- */
static uint32_t bt_app_hf_outgoing_cb(uint8_t *p_buf, uint32_t sz)
{
    return walkie_talkie_audio_to_headset(p_buf, sz);
}

/* ---- Incoming callback: route headset mic audio to Codec2 → ESP-NOW ---- */
static void bt_app_hf_incoming_cb(const uint8_t *buf, uint32_t sz)
{
    walkie_talkie_audio_from_headset(buf, sz);
}

/* ---- Timer: periodically prompt HFP to pull speaker audio -------------- */
static void bt_app_outgoing_timer_cb(void *arg)
{
    uint64_t now      = esp_timer_get_time();
    uint64_t duration = now - s_last_enter_time;
    uint32_t block    = (s_audio_code == ESP_HF_AUDIO_STATE_CONNECTED_MSBC)
                        ? WBS_PCM_INPUT_DATA_SIZE : PCM_INPUT_DATA_SIZE;
    if (duration >= PCM_BLOCK_DURATION_US) {
        s_last_enter_time += (duration / PCM_BLOCK_DURATION_US) * PCM_BLOCK_DURATION_US;
        /* Signal the HFP driver to call bt_app_hf_outgoing_cb() */
        esp_hf_ag_outgoing_data_ready();
        (void)block;
    }
}

void bt_app_send_data(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = bt_app_outgoing_timer_cb,
        .name     = "hfp_out"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_outgoing_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_outgoing_timer, PCM_GENERATOR_TICK_US));
    s_last_enter_time = esp_timer_get_time();
}

void bt_app_send_data_shut_down(void)
{
    if (s_outgoing_timer) {
        esp_timer_stop(s_outgoing_timer);
        esp_timer_delete(s_outgoing_timer);
        s_outgoing_timer = NULL;
    }
}

#endif /* CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI */

/* ---- Async retry: remove stale bond after delay, then reconnect ------- */
static esp_timer_handle_t s_retry_timer = NULL;

static void bt_app_hf_retry_timer_cb(void *arg)
{
    ESP_LOGW(BT_HF_TAG, "Retry: removing stale bond & reconnecting...");
    esp_bt_gap_remove_bond_device(hf_peer_addr);
    /* Small yield to let bond removal propagate */
    esp_hf_ag_slc_connect(hf_peer_addr);
}

static void bt_app_hf_schedule_retry(void)
{
    if (!s_retry_timer) {
        const esp_timer_create_args_t args = {
            .callback = bt_app_hf_retry_timer_cb,
            .name     = "hfp_retry"
        };
        esp_timer_create(&args, &s_retry_timer);
    }
    /* Fire after 1 second – gives ACL time to fully tear down */
    esp_timer_start_once(s_retry_timer, 1000000ULL);  /* 1 000 000 µs = 1 s */
}

void bt_app_hf_cb(esp_hf_cb_event_t event, esp_hf_cb_param_t *param)
{
    static bool s_slc_reached = false;   /* true once SLC_CONNECTED seen */
    static int  s_retry_count = 0;       /* avoid infinite retry loop   */

    if (event <= ESP_HF_PKT_STAT_NUMS_GET_EVT) {
        ESP_LOGW(BT_HF_TAG, "APP HFP event: %s (id=%d)", c_hf_evt_str[event], event);
    } else {
        ESP_LOGE(BT_HF_TAG, "APP HFP invalid event %d", event);
    }

    switch (event) {
        case ESP_HF_CONNECTION_STATE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--connection state %s, peer feats 0x%"PRIx32", chld_feats 0x%"PRIx32,
                    c_connection_state_str[param->conn_stat.state],
                    param->conn_stat.peer_feat,
                    param->conn_stat.chld_feat);
            memcpy(hf_peer_addr, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);

            if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_CONNECTED) {
                /* ACL up but SLC not yet – remember we haven't reached SLC */
                s_slc_reached = false;
                /* Reset retry counter on each new connection attempt */
                s_retry_count = 0;
            }
            /* Auto-open SCO audio once SLC is fully established */
            else if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_SLC_CONNECTED) {
                s_slc_reached = true;
                s_retry_count = 0;
                ESP_LOGI(BT_HF_TAG, "SLC up – auto-connecting audio (SCO)...");
                esp_hf_ag_audio_connect(hf_peer_addr);
            } else if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_DISCONNECTED) {
                walkie_talkie_on_audio_state(false, false);

                /* If we never reached SLC, the RFCOMM channel likely failed
                 * because of stale bond keys (headset was re-paired / reset).
                 * Remove the old bond and retry – fresh SSP will kick in.
                 * Dispatch async so we don't block the BT callback task. */
                if (!s_slc_reached && s_retry_count < 2) {
                    s_retry_count++;
                    ESP_LOGW(BT_HF_TAG, "SLC never reached – scheduling retry (%d/2)...",
                             s_retry_count);
                    bt_app_hf_schedule_retry();
                }
            }
            break;
        }

        case ESP_HF_AUDIO_STATE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Audio State %s", c_audio_state_str[param->audio_stat.state]);
#if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI
            if (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED ||
                param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED_MSBC)
            {
                bool is_wbs = (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED_MSBC);
                s_audio_code = param->audio_stat.state;
                esp_hf_ag_register_data_callback(bt_app_hf_incoming_cb, bt_app_hf_outgoing_cb);
                bt_app_send_data();
                walkie_talkie_on_audio_state(true, is_wbs);

                /* Tell headset "call is active" so the hook button sends
                   AT+CHUP (hang-up), which we intercept for PTT toggle. */
                esp_hf_ag_ciev_report(hf_peer_addr,
                                      ESP_HF_IND_TYPE_CALL, 1);
            } else if (param->audio_stat.state == ESP_HF_AUDIO_STATE_DISCONNECTED) {
                ESP_LOGI(BT_HF_TAG, "--ESP AG Audio Connection Disconnected.");
                bt_app_send_data_shut_down();
                walkie_talkie_on_audio_state(false, false);
            }
#endif /* CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI */
            break;
        }

        case ESP_HF_BVRA_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Voice Recognition is %s", c_vr_state_str[param->vra_rep.value]);
            break;
        }

        case ESP_HF_VOLUME_CONTROL_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Volume Target: %s, Volume %d",
                     c_volume_control_target_str[param->volume_control.type],
                     param->volume_control.volume);
            walkie_talkie_on_volume(param->volume_control.type,
                                    param->volume_control.volume);
            break;
        }

        case ESP_HF_UNAT_RESPONSE_EVT:
        {
            ESP_LOGW(BT_HF_TAG, "--UNKNOWN AT CMD from headset: \"%s\"", param->unat_rep.unat);
            esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr, NULL);
            break;
        }

        case ESP_HF_IND_UPDATE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--UPDATE INDCATOR!");
            esp_hf_call_status_t call_state = 1;
            esp_hf_call_setup_status_t call_setup_state = 2;
            esp_hf_network_state_t ntk_state = 1;
            int signal = 2;
            int battery = 3;
            esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_CALL, call_state);
            esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_CALLSETUP, call_setup_state);
            esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_SERVICE, ntk_state);
            esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_SIGNAL, signal);
            esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_BATTCHG, battery);
            break;
        }

        case ESP_HF_CIND_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--CIND Start.");
            esp_hf_call_status_t call_status = 0;
            esp_hf_call_setup_status_t call_setup_status = 0;
            esp_hf_network_state_t ntk_state = 1;
            int signal = 4;
            esp_hf_roaming_status_t roam = 0;
            int batt_lev = 3;
            esp_hf_call_held_status_t call_held_status = 0;
            esp_hf_ag_cind_response(param->cind_rep.remote_addr,call_status,call_setup_status,ntk_state,signal,roam,batt_lev,call_held_status);
            break;
        }

        case ESP_HF_COPS_RESPONSE_EVT:
        {
            const int svc_type = 1;
            esp_hf_ag_cops_response(param->cops_rep.remote_addr, c_operator_name_str[svc_type]);
            break;
        }

        case ESP_HF_CLCC_RESPONSE_EVT:
        {
            int index = 1;
            //mandatory
            esp_hf_current_call_direction_t dir = 1;
            esp_hf_current_call_status_t current_call_status = 0;
            esp_hf_current_call_mode_t mode = 0;
            esp_hf_current_call_mpty_type_t mpty = 0;
            //option
            char *number = {"123456"};
            esp_hf_call_addr_type_t type = ESP_HF_CALL_ADDR_TYPE_UNKNOWN;

            ESP_LOGI(BT_HF_TAG, "--Calling Line Identification.");
            esp_hf_ag_clcc_response(param->clcc_rep.remote_addr, index, dir, current_call_status, mode, mpty, number, type);

            //AG shall always send ok response to HF
            //index = 0 means response ok
            index = 0;
            esp_hf_ag_clcc_response(param->clcc_rep.remote_addr, index, dir, current_call_status, mode, mpty, number, type);
            break;
        }

        case ESP_HF_CNUM_RESPONSE_EVT:
        {
            char *number = {"123456"};
            int number_type = 129;
            esp_hf_subscriber_service_type_t service_type = ESP_HF_SUBSCRIBER_SERVICE_TYPE_VOICE;
            if (service_type == ESP_HF_SUBSCRIBER_SERVICE_TYPE_VOICE || service_type == ESP_HF_SUBSCRIBER_SERVICE_TYPE_FAX) {
                ESP_LOGI(BT_HF_TAG, "--Current Number is %s, Number Type is %d, Service Type is %s.", number, number_type, c_subscriber_service_type_str[service_type - 3]);
            } else {
                ESP_LOGI(BT_HF_TAG, "--Current Number is %s, Number Type is %d, Service Type is %s.", number, number_type, c_subscriber_service_type_str[0]);
            }
            esp_hf_ag_cnum_response(hf_peer_addr, number, number_type, service_type);
            break;
        }

        case ESP_HF_VTS_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--DTMF code is: %s.", param->vts_rep.code);
            break;
        }

        case ESP_HF_NREC_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--NREC status is: %s.", c_nrec_status_str[param->nrec.state]);
            break;
        }

        case ESP_HF_ATA_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Answer/PTT toggle");
            walkie_talkie_toggle_ptt();
            /* Keep headset happy: send OK response */
            esp_hf_ag_cmee_send(param->ata_rep.remote_addr,
                                ESP_HF_AT_RESPONSE_CODE_OK, ESP_HF_CME_AG_FAILURE);
            break;
        }

        case ESP_HF_CHUP_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Hangup/PTT toggle");
            walkie_talkie_toggle_ptt();
            /* Respond OK but re-assert call=1 so the headset keeps the
               "active call" state and the hook button stays functional. */
            esp_hf_ag_cmee_send(param->chup_rep.remote_addr,
                                ESP_HF_AT_RESPONSE_CODE_OK, ESP_HF_CME_AG_FAILURE);
            esp_hf_ag_ciev_report(param->chup_rep.remote_addr,
                                  ESP_HF_IND_TYPE_CALL, 1);
            break;
        }

        case ESP_HF_DIAL_EVT:
        {
            if (param->out_call.num_or_loc) {
                /* Actual dial with number/memory – ignore for PTT */
                ESP_LOGI(BT_HF_TAG, "--Dial \"%s\" (ignored)",
                         param->out_call.num_or_loc);
                esp_hf_ag_cmee_send(param->out_call.remote_addr,
                                    ESP_HF_AT_RESPONSE_CODE_OK, ESP_HF_CME_AG_FAILURE);
            } else {
                /* Dial-last (AT+BLDN) – headset button when idle → PTT toggle */
                ESP_LOGI(BT_HF_TAG, "--Dial-last/PTT toggle");
                walkie_talkie_toggle_ptt();
                esp_hf_ag_cmee_send(param->out_call.remote_addr,
                                    ESP_HF_AT_RESPONSE_CODE_OK, ESP_HF_CME_AG_FAILURE);
            }
            break;
        }
#if (CONFIG_BT_HFP_WBS_ENABLE)
        case ESP_HF_WBS_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Current codec: %s",c_codec_mode_str[param->wbs_rep.codec]);
            break;
        }
#endif
        case ESP_HF_BCS_RESPONSE_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "--Consequence of codec negotiation: %s",c_codec_mode_str[param->bcs_rep.mode]);
            break;
        }
        case ESP_HF_PKT_STAT_NUMS_GET_EVT:
        {
            ESP_LOGI(BT_HF_TAG, "ESP_HF_PKT_STAT_NUMS_GET_EVT: %d.", event);
            break;
        }

        default:
            ESP_LOGI(BT_HF_TAG, "Unsupported HF_AG EVT: %d.", event);
            break;

    }
}
