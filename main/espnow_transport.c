/*
 * ESP-NOW Audio Transport – implementation
 */

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_coexist.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "espnow_transport.h"

static const char *TAG = "ESPNOW";

/* Rolling sequence counter */
static uint16_t s_tx_seq = 0;

/* Statistics */
static uint32_t s_tx_errors = 0;
static uint32_t s_rx_count  = 0;

/* User receive callback */
static espnow_rx_cb_t s_rx_cb = NULL;

/* Current peer MAC (stored for channel/peer changes and RX filtering) */
static uint8_t s_peer_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* Broadcast address for fire-and-forget sending (no ACK required).
 * Unicast ESP-NOW requires peer ACK, which fails ~100% when both
 * devices run BT SCO because coexistence starves WiFi of radio time. */
static const uint8_t s_bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static bool is_broadcast(const uint8_t *mac)
{
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0xFF) return false;
    }
    return true;
}

static void s_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        s_tx_errors++;
    }
}

static void s_recv_cb(const esp_now_recv_info_t *info,
                      const uint8_t             *data,
                      int                        data_len)
{
    if (data_len < (int)ESPNOW_HEADER_LEN) {
        ESP_LOGW(TAG, "RX packet too short (%d bytes)", data_len);
        return;
    }

    const espnow_audio_pkt_t *pkt = (const espnow_audio_pkt_t *)data;

    if (pkt->magic != ESPNOW_PKT_MAGIC) {
        /* Not our audio packet – silently drop */
        return;
    }

    if (data_len < (int)(ESPNOW_HEADER_LEN + pkt->len)) {
        ESP_LOGW(TAG, "RX packet truncated");
        return;
    }

    s_rx_count++;

    if (s_rx_cb) {
        s_rx_cb(info->src_addr, pkt->payload, pkt->len, pkt->codec, pkt->seq);
    }
}

/* -----------------------------------------------------------------------
 * Add or update an ESP-NOW peer entry
 * ----------------------------------------------------------------------- */
static esp_err_t add_or_update_peer(const uint8_t *mac, uint8_t channel)
{
    esp_now_peer_info_t peer = {
        .channel   = 0,        /* 0 = use current channel (avoids coex issues) */
        .encrypt   = false,
        .ifidx     = WIFI_IF_STA,
    };
    (void)channel;
    memcpy(peer.peer_addr, mac, 6);

    if (esp_now_is_peer_exist(mac)) {
        return esp_now_mod_peer(&peer);
    } else {
        return esp_now_add_peer(&peer);
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

esp_err_t espnow_transport_init(uint8_t        channel,
                                const uint8_t *peer_mac,
                                espnow_rx_cb_t rx_cb)
{
    esp_err_t ret;

    s_rx_cb = rx_cb;
    memcpy(s_peer_mac, peer_mac, 6);

    /* ---- Wi-Fi init (station, no actual connection needed) ---- */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); /* disable power-save for low latency */

    /* NOTE: Do NOT call esp_coex_preference_set(ESP_COEX_PREFER_BALANCE) here.
     * BT SCO (eSCO) is a synchronous link that needs guaranteed radio slots.
     * Giving WiFi equal priority starves SCO and breaks the headset audio
     * pipe entirely.  The default BT-priority policy is correct. */

    /* ---- ESP-NOW ---- */
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(esp_now_register_send_cb(s_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(s_recv_cb));

    /* Add peer */
    ret = add_or_update_peer(s_peer_mac, channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Always add broadcast peer for fire-and-forget sending */
    if (!is_broadcast(s_peer_mac)) {
        add_or_update_peer(s_bcast, channel);
    }

    ESP_LOGI(TAG, "Initialised on ch %d, peer " MACSTR " (broadcast TX)",
             channel, MAC2STR(s_peer_mac));
    return ESP_OK;
}

esp_err_t espnow_transport_set_channel(uint8_t channel)
{
    esp_err_t ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_channel(%d) failed: %s", channel, esp_err_to_name(ret));
        return ret;
    }
    /* Update peer channel */
    esp_now_peer_info_t peer;
    if (esp_now_get_peer(s_peer_mac, &peer) == ESP_OK) {
        peer.channel = channel;
        esp_now_mod_peer(&peer);
    }
    ESP_LOGI(TAG, "Channel changed to %d", channel);
    return ESP_OK;
}

esp_err_t espnow_transport_set_peer(const uint8_t *peer_mac)
{
    /* Remove old unicast peer if it differs */
    if (!is_broadcast(s_peer_mac) && memcmp(s_peer_mac, peer_mac, 6) != 0) {
        esp_now_del_peer(s_peer_mac);
    }

    memcpy(s_peer_mac, peer_mac, 6);

    uint8_t channel = 1;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&channel, &second);

    esp_err_t ret = add_or_update_peer(s_peer_mac, channel);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Peer updated to " MACSTR, MAC2STR(s_peer_mac));
    }
    return ret;
}

esp_err_t espnow_transport_send(const uint8_t *payload,
                                uint8_t        payload_len,
                                uint8_t        codec_mode)
{
    if (payload_len > ESPNOW_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Build packet on stack – max 250 bytes total */
    uint8_t buf[ESPNOW_HEADER_LEN + ESPNOW_MAX_PAYLOAD];
    espnow_audio_pkt_t *pkt = (espnow_audio_pkt_t *)buf;

    pkt->magic = ESPNOW_PKT_MAGIC;
    pkt->codec  = codec_mode;
    pkt->seq    = s_tx_seq++;
    pkt->len    = payload_len;
    memcpy(pkt->payload, payload, payload_len);

    /* Broadcast send: fire-and-forget – no ACK required.  This avoids the
     * near-100% TX failure rate caused by BT SCO + WiFi coexistence
     * starving the peer of radio time to ACK unicast frames. */
    esp_err_t ret = esp_now_send(s_bcast, buf, ESPNOW_HEADER_LEN + payload_len);
    if (ret != ESP_OK) {
        s_tx_errors++;
    }
    return ret;
}

void espnow_transport_deinit(void)
{
    esp_now_deinit();
    esp_wifi_stop();
    esp_wifi_deinit();
}

uint32_t espnow_transport_tx_errors(void)
{
    return s_tx_errors;
}

uint32_t espnow_transport_rx_count(void)
{
    return s_rx_count;
}
