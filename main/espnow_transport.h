/*
 * ESP-NOW Audio Transport
 *
 * Thin ESP-NOW wrapper that sends / receives Codec2-encoded (or raw PCM)
 * audio packets between two ESP32 walkie-talkie units.
 *
 * Packet format (max 250 bytes total):
 *   [magic:1][codec:1][seq:2][len:1][payload:N]  (5-byte header)
 *
 * codec values (match config.h Codec2 mode IDs):
 *   0 = CODEC2_MODE_3200 …  9 = CODEC2_MODE_ADPCM
 *   0xFF = RAW_PCM8 (fallback / passthrough, 8-bit linear packed)
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* -----------------------------------------------------------------------
 * Packet framing
 * ----------------------------------------------------------------------- */

#define ESPNOW_PKT_MAGIC   0xABU

/* Packet header – packed, no padding. */
typedef struct __attribute__((packed)) {
    uint8_t  magic;   /* ESPNOW_PKT_MAGIC                        */
    uint8_t  codec;   /* Codec2 mode id (0-8) or 0xFF = raw PCM  */
    uint16_t seq;     /* Rolling sequence counter                 */
    uint8_t  len;     /* Payload bytes that follow                */
    uint8_t  payload[]; /* Encoded audio data                    */
} espnow_audio_pkt_t;

#define ESPNOW_HEADER_LEN  (sizeof(espnow_audio_pkt_t))  /* 5 bytes */
#define ESPNOW_MAX_PAYLOAD 245U

/* -----------------------------------------------------------------------
 * Callback type – called from ISR/WiFi task, keep it short!
 * The implementation must call espnow_transport_copy_rx if it needs to
 * keep the data past the callback.
 * ----------------------------------------------------------------------- */
typedef void (*espnow_rx_cb_t)(const uint8_t *src_mac,
                                const uint8_t *payload,
                                uint8_t        payload_len,
                                uint8_t        codec_mode,
                                uint16_t       seq);

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/**
 * @brief Initialise Wi-Fi (station mode, no connect) and ESP-NOW.
 *
 * Must be called AFTER nvs_flash_init() and BEFORE any send/recv calls.
 * The Bluetooth stack must already be enabled (BT+WiFi coexistence).
 *
 * @param channel   Wi-Fi channel for ESP-NOW (1-13)
 * @param peer_mac  6-byte MAC of the remote ESP32 (use broadcast FF:FF:…
 *                  if the peer address is not yet known)
 * @param rx_cb     Callback invoked for every valid received audio packet
 * @return          ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t espnow_transport_init(uint8_t        channel,
                                const uint8_t *peer_mac,
                                espnow_rx_cb_t rx_cb);

/**
 * @brief Change the ESP-NOW Wi-Fi channel at runtime.
 * @param channel  1-13
 */
esp_err_t espnow_transport_set_channel(uint8_t channel);

/**
 * @brief Update the peer MAC address (unicast destination) at runtime.
 *        Pass {0xFF,…} for broadcast.
 * @param peer_mac  6-byte MAC
 */
esp_err_t espnow_transport_set_peer(const uint8_t *peer_mac);

/**
 * @brief Send an audio payload via ESP-NOW.
 *
 * Prepends the 5-byte header automatically.
 *
 * @param payload     Encoded audio bytes
 * @param payload_len Number of bytes (max ESPNOW_MAX_PAYLOAD)
 * @param codec_mode  Codec2 mode id used to encode (0-8) or 0xFF=raw
 * @return            ESP_OK / ESP_ERR_INVALID_ARG / ESP_FAIL
 */
esp_err_t espnow_transport_send(const uint8_t *payload,
                                uint8_t        payload_len,
                                uint8_t        codec_mode);

/**
 * @brief De-initialise ESP-NOW and release Wi-Fi.
 */
void espnow_transport_deinit(void);

/**
 * @brief Return number of TX send errors since init (for 'status' command).
 */
uint32_t espnow_transport_tx_errors(void);

/**
 * @brief Return total RX packets received since init.
 */
uint32_t espnow_transport_rx_count(void);

#ifdef __cplusplus
}
#endif
