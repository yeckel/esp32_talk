/*
 * Walkie-Talkie Core
 *
 * Orchestrates:
 *   - Bluetooth HFP Audio Gateway connection to a headset
 *   - Codec2 encode/decode of PCM audio
 *   - ESP-NOW duplex transport to the remote ESP32 unit
 *   - NVS persistence of configuration (headset MAC, peer MAC, channel, mode)
 *   - Runtime status reporting
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_bt_defs.h"

/* -----------------------------------------------------------------------
 * State
 * ----------------------------------------------------------------------- */
typedef enum {
    WT_STATE_IDLE           = 0,  /* BT stack ready, not connected        */
    WT_STATE_BT_CONNECTING  = 1,  /* SLC connection in progress           */
    WT_STATE_BT_CONNECTED   = 2,  /* SLC connected, no audio              */
    WT_STATE_AUDIO_ACTIVE   = 3,  /* Audio (eSCO/SCO) connection up       */
    WT_STATE_ERROR          = 4,
} wt_state_t;

/* -----------------------------------------------------------------------
 * Configuration (all fields persisted to NVS)
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t headset_mac[6];   /* BT MAC of the headset                   */
    uint8_t peer_mac[6];      /* Wi-Fi MAC of the remote ESP32 unit       */
    uint8_t espnow_channel;   /* Wi-Fi channel for ESP-NOW  (1-13)        */
    int     codec2_mode;      /* Codec2 mode id  (CODEC2_MODE_*)          */
} wt_config_t;

/* -----------------------------------------------------------------------
 * Statistics
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t tx_frames;       /* Codec2 frames encoded and sent           */
    uint32_t rx_frames;       /* Codec2 frames received and decoded       */
    uint32_t tx_errors;       /* ESP-NOW send failures                    */
    uint32_t rx_lost;         /* Estimated lost/out-of-order RX frames    */
    uint32_t bt_audio_drop;   /* TX PCM bytes dropped (ring-buf overflow) */
    int      speaker_vol;     /* Last known headset speaker volume (0-15) */
    int      mic_vol;         /* Last known headset mic volume     (0-15) */
} wt_stats_t;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Load config from NVS, initialise Codec2 and ESP-NOW, prepare BT.
 *         Call once after the BT stack is enabled and NVS is initialised.
 */
esp_err_t walkie_talkie_init(void);

/**
 * @brief  Read the active configuration.
 */
void walkie_talkie_get_config(wt_config_t *cfg);

/**
 * @brief  Persist updated configuration to NVS and apply changes at runtime.
 *         Fields set to all-zero / default are left unchanged.
 */
esp_err_t walkie_talkie_set_config(const wt_config_t *cfg);

/**
 * @brief  Set headset MAC and save to NVS.
 */
esp_err_t walkie_talkie_set_headset_mac(const uint8_t mac[6]);

/**
 * @brief  Set peer ESP32 MAC for ESP-NOW and save to NVS.
 */
esp_err_t walkie_talkie_set_peer_mac(const uint8_t mac[6]);

/**
 * @brief  Change ESP-NOW Wi-Fi channel and save to NVS.
 */
esp_err_t walkie_talkie_set_channel(uint8_t channel);

/**
 * @brief  Change Codec2 bandwidth mode and save to NVS.
 *         Takes effect immediately (re-creates Codec2 state).
 */
esp_err_t walkie_talkie_set_codec2_mode(int mode);

/**
 * @brief  Current state of the walkie-talkie.
 */
wt_state_t walkie_talkie_get_state(void);

/**
 * @brief  Human-readable name for a wt_state_t value.
 */
const char *walkie_talkie_state_name(wt_state_t state);

/**
 * @brief  Snapshot of audio / transport statistics.
 */
void walkie_talkie_get_stats(wt_stats_t *stats);

/**
 * @brief  Reset all counters in the statistics structure.
 */
void walkie_talkie_reset_stats(void);

/**
 * @brief  Called from bt_app_hf.c when a PCM block arrives from the
 *         headset microphone (audio going OUT to the peer unit).
 *
 * Thread-safe; may be called from any task.
 *
 * @param buf  PCM data (int16_t samples, native-endian)
 * @param len  Number of bytes
 */
void walkie_talkie_audio_from_headset(const uint8_t *buf, uint32_t len);

/**
 * @brief  Called from bt_app_hf.c when the HFP driver needs PCM data to
 *         send to the headset speaker (audio coming IN from the peer unit).
 *
 * Returns the number of bytes actually written into buf (may be 0 if no
 * decoded audio is available yet – the HFP driver interprets 0 as silence).
 *
 * @param buf  Destination buffer
 * @param len  Number of bytes requested
 * @return     Bytes filled (0..len)
 */
uint32_t walkie_talkie_audio_to_headset(uint8_t *buf, uint32_t len);

/**
 * @brief  Notify the WT core that audio connection state has changed.
 *         Called from the HFP callback.
 */
void walkie_talkie_on_audio_state(bool audio_up, bool is_wbs);

/**
 * @brief  Notify the WT core of a speaker/mic volume change from the headset.
 */
void walkie_talkie_on_volume(int target, int volume);

/**
 * @brief  Set VOX threshold. 0 = disable VOX (PTT-only mode).
 *         Typical values: 100-500 for quiet rooms, 500-2000 for noisy.
 */
void walkie_talkie_set_vox_threshold(uint16_t threshold);

/**
 * @brief  Toggle PTT (push-to-talk).  While PTT is active, VOX gating is
 *         bypassed and all audio is transmitted.  Called from the HFP
 *         callback when the headset play/pause button is pressed.
 */
void walkie_talkie_toggle_ptt(void);

/**
 * @brief  Return current PTT state.
 */
bool walkie_talkie_get_ptt(void);

/**
 * @brief  Print a formatted status report to stdout (for the 'status' command).
 */
void walkie_talkie_print_status(void);

#ifdef __cplusplus
}
#endif
