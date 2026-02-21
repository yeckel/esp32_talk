/*
 * Walkie-Talkie Core – implementation
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "config.h"
#include "walkie_talkie.h"
#include "espnow_transport.h"
#include "codec2_wrapper.h"
#include "display.h"

static const char *TAG = "WT";

/* =========================================================================
 * Internal state
 * ========================================================================= */

static wt_state_t      s_state      = WT_STATE_IDLE;
static wt_config_t     s_config;
static wt_stats_t      s_stats;
static SemaphoreHandle_t s_mutex    = NULL;
static bool            s_audio_up   = false;
static bool            s_is_wbs     = false;  /* Wide-Band Speech (mSBC/16kHz) */

/* TX: PCM from headset mic → Codec2 encoder → ESP-NOW */
static RingbufHandle_t  s_tx_rb     = NULL;
/* RX: ESP-NOW → Codec2 decoder → PCM to headset speaker */
static RingbufHandle_t  s_rx_rb     = NULL;

/* Dedicated TX task – does encode + batch + ESP-NOW send off the BT task */
static TaskHandle_t     s_tx_task   = NULL;

/* Dedicated RX task – decodes Codec2 off the WiFi callback (whose stack is
 * only ~3.5 KB, far too small for the Codec2 decode call-chain). */
static TaskHandle_t     s_rx_task   = NULL;
/* Encoded-frame ring buffer: ESP-NOW RX callback → wt_rx_task */
static RingbufHandle_t  s_rx_enc_rb = NULL;

/* Scratchpad buffers (used only by TX task) */
static int16_t  s_enc_buf[320];      /* max Codec2 frame size (samples) */
static uint8_t  s_bits_buf[64];      /* max Codec2 encoded bytes */

/* VOX (voice-activated transmission) */
static uint16_t s_vox_threshold = DEFAULT_VOX_THRESHOLD;
static uint16_t s_vox_hangtime  = VOX_HANGTIME_FRAMES;
static uint16_t s_vox_hang_ctr  = 0;   /* frames remaining in hang period */
static bool     s_vox_active    = false; /* true while transmitting */

/* VOX look-back ring: keeps the last N encoded frames so that the onset
 * consonant that *triggered* VOX is not lost.  When VOX transitions from
 * silent→active we flush the ring into the batch buffer first. */
#define VOX_LOOKBACK_FRAMES  3   /* ~60 ms at bw 0 (20 ms/frame) */
static uint8_t  s_vox_lb_buf[VOX_LOOKBACK_FRAMES][64]; /* max bpf */
static int      s_vox_lb_len[VOX_LOOKBACK_FRAMES];     /* encoded bytes */
static int      s_vox_lb_wr   = 0;   /* write index */
static int      s_vox_lb_cnt  = 0;   /* frames stored */
static bool     s_vox_was_active = false; /* previous state for edge detect */

/* PTT (push-to-talk) – overrides VOX when active */
static bool     s_ptt_active    = false;

/* =========================================================================
 * NVS helpers
 * ========================================================================= */

static esp_err_t nvs_load_config(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not found – using defaults");
        return ESP_OK;
    }
    ESP_ERROR_CHECK(ret);

    size_t len;

    len = 6;
    nvs_get_blob(h, NVS_KEY_HEADSET_MAC, s_config.headset_mac, &len);

    len = 6;
    nvs_get_blob(h, NVS_KEY_PEER_MAC, s_config.peer_mac, &len);

    uint8_t ch = 0;
    if (nvs_get_u8(h, NVS_KEY_ESPNOW_CHANNEL, &ch) == ESP_OK && ch >= 1 && ch <= 13) {
        s_config.espnow_channel = ch;
    }

    int32_t mode = 0;
    if (nvs_get_i32(h, NVS_KEY_CODEC2_MODE, &mode) == ESP_OK) {
        s_config.codec2_mode = (int)mode;
    }

    uint16_t vox = 0;
    if (nvs_get_u16(h, NVS_KEY_VOX_THRESHOLD, &vox) == ESP_OK) {
        s_vox_threshold = vox;
    }

    nvs_close(h);
    return ESP_OK;
}

static esp_err_t nvs_save_config(void)
{
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h));
    nvs_set_blob(h, NVS_KEY_HEADSET_MAC, s_config.headset_mac, 6);
    nvs_set_blob(h, NVS_KEY_PEER_MAC,    s_config.peer_mac,    6);
    nvs_set_u8  (h, NVS_KEY_ESPNOW_CHANNEL, s_config.espnow_channel);
    nvs_set_i32 (h, NVS_KEY_CODEC2_MODE,    (int32_t)s_config.codec2_mode);
    nvs_set_u16 (h, NVS_KEY_VOX_THRESHOLD,   s_vox_threshold);
    esp_err_t ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}

/* =========================================================================
 * ESP-NOW receive callback
 * ========================================================================= */

/*
 * ESP-NOW receive callback – runs on the WiFi task (~3.5 KB stack).
 * MUST be lightweight: no Codec2 calls.  Just enqueue raw encoded
 * payload for wt_rx_task to decode on its own (larger) stack.
 */
static void espnow_rx_handler(const uint8_t *src_mac,
                               const uint8_t *payload,
                               uint8_t        payload_len,
                               uint8_t        codec_mode,
                               uint16_t       seq)
{
    if (!s_rx_enc_rb || payload_len == 0) return;

    /* Item layout: [seq_lo, seq_hi, encoded_payload...] */
    uint8_t item[2 + ESPNOW_MAX_PAYLOAD];
    item[0] = (uint8_t)(seq & 0xFF);
    item[1] = (uint8_t)(seq >> 8);
    memcpy(item + 2, payload, payload_len);

    BaseType_t ok = xRingbufferSend(s_rx_enc_rb, item, 2 + payload_len, 0);
    if (!ok) {
        ESP_LOGV(TAG, "RX enc ring-buf full, dropping packet");
    }
}

/* =========================================================================
 * Forward declarations
 * ========================================================================= */
static void wt_tx_task(void *arg);
static void wt_rx_task(void *arg);

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t walkie_talkie_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    /* Defaults */
    uint8_t def_hs_mac[]   = DEFAULT_HEADSET_MAC;
    uint8_t def_peer_mac[] = DEFAULT_ESPNOW_PEER_MAC;
    memcpy(s_config.headset_mac,   def_hs_mac,   6);
    memcpy(s_config.peer_mac,      def_peer_mac, 6);
    s_config.espnow_channel = DEFAULT_ESPNOW_CHANNEL;
    s_config.codec2_mode    = DEFAULT_CODEC2_MODE;

    /* Load overrides from NVS */
    nvs_load_config();

    /* Codec2 */
    esp_err_t ret = codec2_wrapper_init(s_config.codec2_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Codec2 init failed");
        return ret;
    }

    /* Ring buffers */
    s_tx_rb     = xRingbufferCreate(WT_TX_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    s_rx_rb     = xRingbufferCreate(WT_RX_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    s_rx_enc_rb = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
    if (!s_tx_rb || !s_rx_rb || !s_rx_enc_rb) {
        ESP_LOGE(TAG, "Failed to create ring buffers");
        return ESP_ERR_NO_MEM;
    }

    /* ESP-NOW */
    ret = espnow_transport_init(s_config.espnow_channel,
                                s_config.peer_mac,
                                espnow_rx_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return ret;
    }

     /* TX task: encode + send on its own stack, never blocks BT callbacks.
         16 KB to keep headroom for codec mode switches and debug builds. */
     xTaskCreate(wt_tx_task, "wt_tx", 16384, NULL, 5, &s_tx_task);

     /* RX task: Codec2 decode on its own stack – the WiFi task that delivers
         ESP-NOW callbacks only has ~3.5 KB, not enough for codec2_decode.
         16 KB to tolerate heavier Codec2 paths (e.g. mode 1) safely. */
     xTaskCreate(wt_rx_task, "wt_rx", 16384, NULL, 5, &s_rx_task);

    /* Display */
    display_init();

    memset(&s_stats, 0, sizeof(s_stats));
    s_state = WT_STATE_IDLE;

    ESP_LOGI(TAG, "Init OK – codec %s, ch %d, free heap %lu",
             codec2_wrapper_mode_name(s_config.codec2_mode),
             s_config.espnow_channel,
             (unsigned long)esp_get_free_heap_size());
    return ESP_OK;
}

void walkie_talkie_get_config(wt_config_t *cfg)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *cfg = s_config;
    xSemaphoreGive(s_mutex);
}

esp_err_t walkie_talkie_set_config(const wt_config_t *cfg)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_config = *cfg;
    xSemaphoreGive(s_mutex);
    return nvs_save_config();
}

esp_err_t walkie_talkie_set_headset_mac(const uint8_t mac[6])
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(s_config.headset_mac, mac, 6);
    xSemaphoreGive(s_mutex);
    esp_err_t ret = nvs_save_config();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Headset MAC updated to " MACSTR, MAC2STR(mac));
    }
    return ret;
}

esp_err_t walkie_talkie_set_peer_mac(const uint8_t mac[6])
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(s_config.peer_mac, mac, 6);
    xSemaphoreGive(s_mutex);
    nvs_save_config();
    return espnow_transport_set_peer(mac);
}

esp_err_t walkie_talkie_set_channel(uint8_t channel)
{
    if (channel < 1 || channel > 13) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_config.espnow_channel = channel;
    xSemaphoreGive(s_mutex);
    nvs_save_config();
    return espnow_transport_set_channel(channel);
}

esp_err_t walkie_talkie_set_codec2_mode(int mode)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_config.codec2_mode = mode;
    xSemaphoreGive(s_mutex);
    nvs_save_config();
    /* Re-init codec2 (safe to call while idle or even during audio, but
       the caller should ideally disconnect audio first) */
    return codec2_wrapper_init(mode);
}

wt_state_t walkie_talkie_get_state(void)
{
    return s_state;
}

const char *walkie_talkie_state_name(wt_state_t state)
{
    switch (state) {
        case WT_STATE_IDLE:          return "IDLE";
        case WT_STATE_BT_CONNECTING: return "BT_CONNECTING";
        case WT_STATE_BT_CONNECTED:  return "BT_CONNECTED";
        case WT_STATE_AUDIO_ACTIVE:  return "AUDIO_ACTIVE";
        case WT_STATE_ERROR:         return "ERROR";
        default:                     return "UNKNOWN";
    }
}

void walkie_talkie_get_stats(wt_stats_t *stats)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *stats = s_stats;
    s_stats.tx_errors = espnow_transport_tx_errors();
    xSemaphoreGive(s_mutex);
}

void walkie_talkie_reset_stats(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(&s_stats, 0, sizeof(s_stats));
    xSemaphoreGive(s_mutex);
}

/* -----------------------------------------------------------------------
 * RX task – decodes Codec2 frames off the WiFi callback stack
 * ----------------------------------------------------------------------- */

static void wt_rx_task(void *arg)
{
    /* Static to keep them off the task stack (saves ~2 KB) */
    static int16_t dec_buf[320];        /* decoded PCM (max Codec2 frame) */
    static int16_t upsample_buf[640];   /* WBS upsample: 320 * 2 */
    static uint16_t s_last_seq = 0;

    ESP_LOGI(TAG, "RX task started");

    for (;;) {
        if (!s_audio_up || !s_rx_enc_rb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t item_size = 0;
        uint8_t *item = xRingbufferReceive(s_rx_enc_rb, &item_size,
                                           pdMS_TO_TICKS(20));
        if (!item || item_size < 3) {   /* need ≥ 2-byte hdr + 1 byte data */
            if (item) vRingbufferReturnItem(s_rx_enc_rb, item);
            continue;
        }

        uint16_t seq = (uint16_t)(item[0] | (item[1] << 8));
        const uint8_t *payload     = item + 2;
        uint8_t        payload_len = (uint8_t)(item_size - 2);

        /* Simple lost-frame estimator */
        uint16_t gap = (uint16_t)(seq - s_last_seq - 1);
        if (s_last_seq != 0 && gap > 0 && gap < 200) {
            s_stats.rx_lost += gap;
        }
        s_last_seq = seq;

        int samples = codec2_wrapper_samples_per_frame();
        int bytes   = codec2_wrapper_bytes_per_frame();

        if (payload_len >= (uint8_t)bytes) {
            int frames_in_packet = payload_len / bytes;
            for (int f = 0; f < frames_in_packet; f++) {
                const uint8_t *frame_bits = payload + f * bytes;

                if (codec2_wrapper_decode(frame_bits, dec_buf) != ESP_OK)
                    continue;

                s_stats.rx_frames++;

                /* Push decoded PCM into RX ring buffer.
                 * Upsample 8 kHz → 16 kHz with 4-point cubic Hermite
                 * interpolation for smoother output than linear.       */
                if (s_is_wbs) {
                    for (int i = 0; i < samples; i++) {
                        /* Four source points clamped at boundaries */
                        int16_t s0 = (i > 0)          ? dec_buf[i - 1] : dec_buf[0];
                        int16_t s1 = dec_buf[i];
                        int16_t s2 = (i < samples - 1) ? dec_buf[i + 1] : dec_buf[samples - 1];
                        int16_t s3 = (i < samples - 2) ? dec_buf[i + 2] : dec_buf[samples - 1];

                        /* Even output sample = original */
                        upsample_buf[i * 2] = s1;

                        /* Odd output sample = cubic Hermite at t=0.5:
                         *   y = (-s0 + 9*s1 + 9*s2 - s3) / 16         */
                        int32_t mid = (-(int32_t)s0 + 9*(int32_t)s1
                                       + 9*(int32_t)s2 - (int32_t)s3) >> 4;
                        if (mid >  32767) mid =  32767;
                        if (mid < -32768) mid = -32768;
                        upsample_buf[i * 2 + 1] = (int16_t)mid;
                    }
                    uint32_t up_bytes = (uint32_t)(samples * 2 * sizeof(int16_t));
                    BaseType_t ok = xRingbufferSend(s_rx_rb, upsample_buf,
                                                    up_bytes, 0);
                    if (!ok) ESP_LOGV(TAG, "RX ring-buf full");
                } else {
                    uint32_t pcm_bytes = (uint32_t)(samples * sizeof(int16_t));
                    BaseType_t ok = xRingbufferSend(s_rx_rb, dec_buf,
                                                    pcm_bytes, 0);
                    if (!ok) ESP_LOGV(TAG, "RX ring-buf full");
                }
            }
        }

        vRingbufferReturnItem(s_rx_enc_rb, item);
        display_update_rx_activity();
    }
}

/* -----------------------------------------------------------------------
 * TX task – runs independently of the BT callback, does encode + send
 * ----------------------------------------------------------------------- */

static void wt_tx_task(void *arg)
{
    ESP_LOGI(TAG, "TX task started");

    for (;;) {
        /* Idle until audio is active */
        if (!s_audio_up || !s_tx_rb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int spf        = codec2_wrapper_samples_per_frame();
        int bpf        = codec2_wrapper_bytes_per_frame();
        int wbs_factor = s_is_wbs ? 2 : 1;

        /* Mode-aware batch size: low-rate Codec2 modes use 2 frames
         * (~25 pps) which was the semi-working baseline; high-rate modes
         * (ADPCM/RAW) use 4 frames to reduce packet rate. */
        int target_batch_frames = WT_TX_BATCH_FRAMES;
        if (s_config.codec2_mode == CODEC2_MODE_RAW ||
            s_config.codec2_mode == CODEC2_MODE_ADPCM) {
            target_batch_frames = WT_TX_BATCH_FRAMES_HI_RATE;
        }
        int max_payload_frames = (bpf > 0) ? ((int)ESPNOW_MAX_PAYLOAD / bpf) : 0;
        if (max_payload_frames < 1) {
            vTaskDelay(1);
            continue;
        }
        if (target_batch_frames > max_payload_frames) {
            target_batch_frames = max_payload_frames;
        }

        uint32_t frame_bytes = (uint32_t)(spf * sizeof(int16_t) * wbs_factor);
        uint32_t batch_pcm   = frame_bytes * (uint32_t)target_batch_frames;

        /* Block until enough PCM is available for a full batch */
        size_t avail = 0;
        vRingbufferGetInfo(s_tx_rb, NULL, NULL, NULL, NULL, &avail);
        if (avail < batch_pcm) {
            vTaskDelay(1);
            continue;
        }

        /* Encode selected batch size and accumulate */
        static uint8_t batch_buf[ESPNOW_MAX_PAYLOAD];
        int     batch_len   = 0;
        int     batch_count = 0;

        for (int f = 0; f < target_batch_frames; f++) {
            size_t got = 0;
            uint8_t *pcm_raw = xRingbufferReceiveUpTo(s_tx_rb, &got, pdMS_TO_TICKS(5), frame_bytes);
            if (!pcm_raw || got < frame_bytes) {
                if (pcm_raw) vRingbufferReturnItem(s_tx_rb, pcm_raw);
                break;
            }

            /* Downsample WBS 16 kHz → 8 kHz with proper anti-alias FIR.
             * 7-tap symmetric lowpass (cutoff ~3.5 kHz, -40 dB at 4 kHz).
             * Coefficients scaled to sum = 16384 (Q14) for integer math.
             * h = [0.025, 0.1, 0.225, 0.3, 0.225, 0.1, 0.025]            */
            static const int16_t lp_fir[7] = {
                410, 1638, 3686, 4916, 3686, 1638, 410
            };
            #define LP_FIR_LEN  7
            #define LP_FIR_HALF 3
            #define LP_FIR_Q    14

            const int16_t *src = (const int16_t *)pcm_raw;
            if (s_is_wbs) {
                /* Filter + decimate by 2.  For each output sample i we
                 * convolve a 7-tap FIR centred on input sample 2*i.
                 * Boundary samples clamp to nearest valid index.       */
                int n_in = spf * 2;  /* samples at 16 kHz */
                for (int i = 0; i < spf; i++) {
                    int centre = i * 2;
                    int32_t acc = 0;
                    for (int t = 0; t < LP_FIR_LEN; t++) {
                        int idx = centre + t - LP_FIR_HALF;
                        if (idx < 0)    idx = 0;
                        if (idx >= n_in) idx = n_in - 1;
                        acc += (int32_t)src[idx] * (int32_t)lp_fir[t];
                    }
                    s_enc_buf[i] = (int16_t)(acc >> LP_FIR_Q);
                }
            } else {
                memcpy(s_enc_buf, src, spf * sizeof(int16_t));
            }
            vRingbufferReturnItem(s_tx_rb, pcm_raw);

            /* --- Encode first, then gate (needed for VOX lookback) --- */
            codec2_wrapper_encode(s_enc_buf, s_bits_buf);

            /* --- TX gate: PTT overrides VOX ----------------------------- */
            if (s_ptt_active) {
                /* PTT held → always transmit, bypass VOX */
            } else if (s_vox_threshold == 0) {
                /* VOX disabled → PTT-only mode, drop frame */
                continue;
            } else {
                /* VOX gating */
                int64_t sum_sq = 0;
                for (int i = 0; i < spf; i++) {
                    int32_t v = s_enc_buf[i];
                    sum_sq += v * v;
                }
                uint32_t rms = 0;
                uint64_t mean = (uint64_t)sum_sq / (uint32_t)spf;
                if (mean > 0) {
                    rms = (uint32_t)mean;
                    uint32_t x = rms;
                    for (int it = 0; it < 6; it++) {
                        if (x == 0) break;
                        x = (x + rms / x) / 2;
                    }
                    rms = x;
                }

                if (rms >= s_vox_threshold) {
                    s_vox_hang_ctr = s_vox_hangtime;
                    s_vox_active = true;
                } else if (s_vox_hang_ctr > 0) {
                    s_vox_hang_ctr--;
                } else {
                    s_vox_active = false;
                }

                if (!s_vox_active) {
                    /* Store encoded frame in lookback ring for onset recovery */
                    memcpy(s_vox_lb_buf[s_vox_lb_wr], s_bits_buf, bpf);
                    s_vox_lb_len[s_vox_lb_wr] = bpf;
                    s_vox_lb_wr = (s_vox_lb_wr + 1) % VOX_LOOKBACK_FRAMES;
                    if (s_vox_lb_cnt < VOX_LOOKBACK_FRAMES) s_vox_lb_cnt++;
                    s_vox_was_active = false;
                    continue;  /* below VOX threshold */
                }
            }
            /* ------------------------------------------------------------- */

            /* VOX just triggered: flush lookback ring (onset consonants) */
            if (!s_vox_was_active && s_vox_lb_cnt > 0) {
                int start = (s_vox_lb_wr - s_vox_lb_cnt + VOX_LOOKBACK_FRAMES)
                            % VOX_LOOKBACK_FRAMES;
                for (int lb = 0; lb < s_vox_lb_cnt; lb++) {
                    int ri = (start + lb) % VOX_LOOKBACK_FRAMES;
                    if (batch_len + s_vox_lb_len[ri] <= (int)sizeof(batch_buf)) {
                        memcpy(batch_buf + batch_len,
                               s_vox_lb_buf[ri], s_vox_lb_len[ri]);
                        batch_len += s_vox_lb_len[ri];
                        batch_count++;
                    }
                }
                s_vox_lb_cnt = 0;
                s_vox_lb_wr  = 0;
            }
            s_vox_was_active = true;

            if (batch_len + bpf <= (int)sizeof(batch_buf)) {
                memcpy(batch_buf + batch_len, s_bits_buf, bpf);
                batch_len += bpf;
                batch_count++;
            }
        }

        if (batch_count > 0) {
            esp_err_t ret = espnow_transport_send(
                batch_buf, (uint8_t)batch_len,
                (uint8_t)s_config.codec2_mode);
            /* One retry on failure – the coexistence arbiter may have
             * blocked the radio momentarily; a brief wait often lets
             * the next attempt through. */
            if (ret != ESP_OK) {
                vTaskDelay(1);
                ret = espnow_transport_send(
                    batch_buf, (uint8_t)batch_len,
                    (uint8_t)s_config.codec2_mode);
            }
            if (ret == ESP_OK) {
                s_stats.tx_frames += batch_count;
                display_update_tx_activity();
            } else {
                s_stats.tx_errors += batch_count;
            }
        }

        /* Yield to prevent watchdog. */
        vTaskDelay(1);
    }
}

/* -----------------------------------------------------------------------
 * Audio routing – called from the HFP task
 * ----------------------------------------------------------------------- */

/*
 * PCM from headset microphone → ring buffer only.
 * Must be non-blocking: encoding and ESP-NOW sending are done by wt_tx_task.
 */
void walkie_talkie_audio_from_headset(const uint8_t *buf, uint32_t len)
{
    if (!s_audio_up || !s_tx_rb) return;

    BaseType_t ok = xRingbufferSend(s_tx_rb, buf, len, 0);
    if (!ok) s_stats.bt_audio_drop += len;
}

/*
 * PCM to headset speaker ← Codec2 decode ← ESP-NOW RX
 * Called from HFP outgoing data callback – must be fast.
 */
uint32_t walkie_talkie_audio_to_headset(uint8_t *buf, uint32_t len)
{
    static int16_t s_last_out_sample = 0;
    static bool s_jb_preroll = true;

    if (!s_rx_rb) {
        int16_t *out = (int16_t *)buf;
        uint32_t samples = len / sizeof(int16_t);
        for (uint32_t i = 0; i < samples; i++) out[i] = s_last_out_sample;
        if (len & 1) buf[len - 1] = 0;
        return len;
    }

    /*
     * Loop to handle ring-buffer wrap-around.  RINGBUF_TYPE_BYTEBUF may
     * return a short read when the internal read pointer hits the end of
     * the circular buffer, even though more data is available after the
     * wrap.  Without looping we'd zero-pad the remainder, creating
     * audible silence glitches every time the buffer wraps (~188 ms).
     */

    /* Jitter-buffer pre-roll: wait until enough PCM is queued before playout. */
    size_t avail = 0;
    vRingbufferGetInfo(s_rx_rb, NULL, NULL, NULL, NULL, &avail);
    uint32_t preroll_bytes = len * WT_JB_PREROLL_BLOCKS;
    if (s_jb_preroll) {
        if (avail < preroll_bytes) {
            int16_t *out = (int16_t *)buf;
            uint32_t samples = len / sizeof(int16_t);
            for (uint32_t i = 0; i < samples; i++) out[i] = s_last_out_sample;
            if (len & 1) buf[len - 1] = 0;
            return len;
        }
        s_jb_preroll = false;
    } else {
        uint32_t rebuffer_bytes = len * WT_JB_REBUFFER_BLOCKS;
        if (avail < rebuffer_bytes) {
            s_jb_preroll = true;
        }
    }

    uint32_t filled = 0;
    while (filled < len) {
        size_t got = 0;
        uint8_t *data = xRingbufferReceiveUpTo(s_rx_rb, &got, 0,
                                               len - filled);
        if (!data || got == 0) break;
        memcpy(buf + filled, data, got);
        vRingbufferReturnItem(s_rx_rb, data);
        filled += got;
    }

    /* Track last valid sample for concealment */
    if (filled >= sizeof(int16_t)) {
        s_last_out_sample = ((int16_t *)buf)[(filled / sizeof(int16_t)) - 1];
    }

    /* Underrun concealment: fade last sample to zero over ~2 ms to avoid
     * a hard DC step when real audio resumes (which sounds like a click).
     * Exponential decay: multiply by 0.9375 (=15/16) each sample. */
    if (filled < len) {
        s_jb_preroll = true;
        int16_t *out = (int16_t *)(buf + filled);
        uint32_t samples = (len - filled) / sizeof(int16_t);
        int32_t fade = (int32_t)s_last_out_sample;
        for (uint32_t i = 0; i < samples; i++) {
            out[i] = (int16_t)fade;
            fade = (fade * 15) >> 4;  /* *= 0.9375 */
        }
        s_last_out_sample = (int16_t)fade;
        if ((len - filled) & 1) buf[len - 1] = 0;
    }
    return len;
}

void walkie_talkie_on_audio_state(bool audio_up, bool is_wbs)
{
    s_audio_up = audio_up;
    s_is_wbs   = is_wbs;

    if (audio_up) {
        s_state = WT_STATE_AUDIO_ACTIVE;
        /* Flush TX ring buffer on audio start */
        if (s_tx_rb) {
            size_t avail;
            vRingbufferGetInfo(s_tx_rb, NULL, NULL, NULL, NULL, &avail);
            if (avail > 0) {
                size_t dummy;
                uint8_t *d = xRingbufferReceiveUpTo(s_tx_rb, &dummy, 0, avail);
                if (d) vRingbufferReturnItem(s_tx_rb, d);
            }
        }
        /* Flush decoded PCM on audio start to force clean jitter-buffer pre-roll */
        if (s_rx_rb) {
            size_t avail;
            vRingbufferGetInfo(s_rx_rb, NULL, NULL, NULL, NULL, &avail);
            if (avail > 0) {
                size_t dummy;
                uint8_t *d = xRingbufferReceiveUpTo(s_rx_rb, &dummy, 0, avail);
                if (d) vRingbufferReturnItem(s_rx_rb, d);
            }
        }
    } else {
        s_state = WT_STATE_BT_CONNECTED;
    }

    display_show_state(s_state);
    ESP_LOGI(TAG, "Audio %s (%s)", audio_up ? "UP" : "DOWN", is_wbs ? "WBS" : "NBS");
}

void walkie_talkie_on_volume(int target, int volume)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (target == 0) s_stats.speaker_vol = volume;   /* 0 = speaker */
    else             s_stats.mic_vol     = volume;   /* 1 = mic     */
    xSemaphoreGive(s_mutex);
}

void walkie_talkie_set_vox_threshold(uint16_t threshold)
{
    s_vox_threshold = threshold;
    s_vox_hang_ctr  = 0;
    s_vox_active    = false;
    nvs_save_config();
    ESP_LOGI(TAG, "VOX threshold set to %u%s", threshold,
             threshold == 0 ? " (disabled – PTT only)" : "");
}

void walkie_talkie_toggle_ptt(void)
{
    s_ptt_active = !s_ptt_active;
    if (!s_ptt_active) {
        /* Reset VOX state when releasing PTT */
        s_vox_hang_ctr = 0;
        s_vox_active   = false;
    }
    ESP_LOGI(TAG, "PTT %s", s_ptt_active ? "ON – transmitting" : "OFF");
}

bool walkie_talkie_get_ptt(void)
{
    return s_ptt_active;
}

void walkie_talkie_print_status(void)
{
    wt_config_t cfg;
    wt_stats_t  st;
    walkie_talkie_get_config(&cfg);
    walkie_talkie_get_stats(&st);

    printf("\n=== Walkie-Talkie Status ===\n");
    printf("  State         : %s\n", walkie_talkie_state_name(s_state));
    printf("  Headset BT MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           cfg.headset_mac[0], cfg.headset_mac[1], cfg.headset_mac[2],
           cfg.headset_mac[3], cfg.headset_mac[4], cfg.headset_mac[5]);
    printf("  Peer ESP32 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           cfg.peer_mac[0], cfg.peer_mac[1], cfg.peer_mac[2],
           cfg.peer_mac[3], cfg.peer_mac[4], cfg.peer_mac[5]);
    printf("  ESP-NOW ch    : %d\n", cfg.espnow_channel);
    printf("  Codec2 mode   : %s (mode id %d)\n",
           codec2_wrapper_mode_name(cfg.codec2_mode), cfg.codec2_mode);
    printf("  Audio path    : %s\n", s_is_wbs ? "WBS 16kHz (mSBC)" : "NBS 8kHz (CVSD)");
    printf("  Speaker vol   : %d/15\n", st.speaker_vol);
    printf("  Mic vol       : %d/15\n", st.mic_vol);
    printf("  PTT           : %s\n", s_ptt_active ? "ON (force TX)" : "OFF");
    printf("  VOX threshold : %u%s\n", s_vox_threshold,
           s_vox_threshold == 0 ? " (OFF – PTT only)" : "");
    printf("  VOX state     : %s\n",
           s_ptt_active ? "BYPASSED (PTT)" :
           s_vox_threshold == 0 ? "OFF (PTT only)" :
           s_vox_active ? "TRANSMITTING" : "silent");
    printf("--- Traffic ---\n");
    printf("  TX frames     : %lu\n", (unsigned long)st.tx_frames);
    printf("  RX frames     : %lu\n", (unsigned long)st.rx_frames);
    printf("  TX errors     : %lu\n", (unsigned long)st.tx_errors);
    printf("  RX lost       : %lu\n", (unsigned long)st.rx_lost);
    printf("  TX PCM drop   : %lu bytes\n", (unsigned long)st.bt_audio_drop);
    printf("  Free heap     : %lu bytes (min %lu)\n",
           (unsigned long)esp_get_free_heap_size(),
           (unsigned long)esp_get_minimum_free_heap_size());
    printf("============================\n\n");
}
