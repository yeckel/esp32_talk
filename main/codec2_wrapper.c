/*
 * Codec2 Wrapper – implementation
 *
 * Selects between the real Codec2 library (when CONFIG_CODEC2_ENABLED is set
 * by the component) and a raw 8-bit PCM passthrough stub.
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "codec2_wrapper.h"

static const char *TAG = "CODEC2";

static SemaphoreHandle_t s_codec_mutex = NULL;

static inline SemaphoreHandle_t codec_lock(void)
{
    SemaphoreHandle_t handle = s_codec_mutex;
    if (handle) xSemaphoreTake(handle, portMAX_DELAY);
    return handle;
}

static inline void codec_unlock(SemaphoreHandle_t handle)
{
    if (handle) xSemaphoreGive(handle);
}

/* ---- IMA ADPCM (4-bit) helpers ---- */
static const int s_ima_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static const int8_t s_ima_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static inline int16_t clamp_s16(int32_t x)
{
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

static inline int clamp_index(int idx)
{
    if (idx < 0) return 0;
    if (idx > 88) return 88;
    return idx;
}

static uint8_t ima_encode_nibble(int16_t sample, int *pred, int *index)
{
    int step = s_ima_step_table[*index];
    int diff = sample - *pred;
    uint8_t code = 0;

    if (diff < 0) {
        code = 8;
        diff = -diff;
    }

    int delta = step >> 3;
    if (diff >= step) { code |= 4; diff -= step; delta += step; }
    step >>= 1;
    if (diff >= step) { code |= 2; diff -= step; delta += step; }
    step >>= 1;
    if (diff >= step) { code |= 1;             delta += step; }

    if (code & 8) *pred -= delta;
    else          *pred += delta;
    *pred = clamp_s16(*pred);

    *index = clamp_index(*index + s_ima_index_table[code & 0x0F]);
    return code & 0x0F;
}

static int16_t ima_decode_nibble(uint8_t code, int *pred, int *index)
{
    int step = s_ima_step_table[*index];
    int delta = step >> 3;
    if (code & 4) delta += step;
    if (code & 2) delta += step >> 1;
    if (code & 1) delta += step >> 2;

    if (code & 8) *pred -= delta;
    else          *pred += delta;
    *pred = clamp_s16(*pred);

    *index = clamp_index(*index + s_ima_index_table[code & 0x0F]);
    return (int16_t)*pred;
}

/* =========================================================================
 * Real Codec2 path
 * ========================================================================= */
#if defined(CONFIG_CODEC2_ENABLED)

#include "codec2.h"

static struct CODEC2 *s_c2_enc = NULL;
static struct CODEC2 *s_c2_dec = NULL;
static int            s_mode   = CODEC2_MODE_3200;
static int16_t        s_raw_prev_dec = 0;
static int            s_adpcm_enc_pred = 0;
static int            s_adpcm_enc_index = 0;

/* ---- RAW passthrough: 8-bit linear PCM (high byte of signed sample) ---- */
#define RAW_SAMPLES 60   /* one CVSD PCM block = 7.5 ms at 8 kHz */
#define RAW_BYTES   60   /* one 8-bit PCM sample per input sample */

/* ---- IMA ADPCM (4-bit) with per-frame state header ----
 * Frame layout: [pred_lo pred_hi idx pad][nibbles...]
 * 60 samples -> 30 nibble bytes + 4-byte header = 34 bytes. */
#define ADPCM_SAMPLES 60
#define ADPCM_BYTES   34

esp_err_t codec2_wrapper_init(int mode)
{
    if (!s_codec_mutex) s_codec_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t lock = codec_lock();

    if (s_c2_enc) { codec2_destroy(s_c2_enc); s_c2_enc = NULL; }
    if (s_c2_dec) { codec2_destroy(s_c2_dec); s_c2_dec = NULL; }

    s_mode = mode;
    s_raw_prev_dec = 0;
    s_adpcm_enc_pred = 0;
    s_adpcm_enc_index = 0;

    if (mode == CODEC2_MODE_RAW) {
        ESP_LOGI(TAG, "Raw 8-bit PCM passthrough mode (no Codec2 compression)");
        codec_unlock(lock);
        return ESP_OK;
    }

    if (mode == CODEC2_MODE_ADPCM) {
        ESP_LOGI(TAG, "IMA ADPCM mode (%d samples/frame, %d bytes/frame)",
                 ADPCM_SAMPLES, ADPCM_BYTES);
        codec_unlock(lock);
        return ESP_OK;
    }

    s_c2_enc = codec2_create(mode);
    s_c2_dec = codec2_create(mode);

    if (!s_c2_enc || !s_c2_dec) {
        ESP_LOGE(TAG, "codec2_create failed for mode %d", mode);
        if (s_c2_enc) { codec2_destroy(s_c2_enc); s_c2_enc = NULL; }
        if (s_c2_dec) { codec2_destroy(s_c2_dec); s_c2_dec = NULL; }
        codec_unlock(lock);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Initialised mode %d (%s), %d samples/frame, %d bytes/frame",
             mode,
             codec2_wrapper_mode_name(mode),
             codec2_samples_per_frame(s_c2_enc),
             (codec2_bits_per_frame(s_c2_enc) + 7) / 8);
    codec_unlock(lock);
    return ESP_OK;
}

esp_err_t codec2_wrapper_encode(const int16_t *pcm_in, uint8_t *bits_out)
{
    SemaphoreHandle_t lock = codec_lock();
    if (s_mode == CODEC2_MODE_RAW) {
        for (int i = 0; i < RAW_SAMPLES; i++)
            bits_out[i] = (uint8_t)(((int32_t)pcm_in[i] + 32768) >> 8);
        codec_unlock(lock);
        return ESP_OK;
    }
    if (s_mode == CODEC2_MODE_ADPCM) {
        int pred = s_adpcm_enc_pred;
        int idx  = s_adpcm_enc_index;
        bits_out[0] = (uint8_t)(pred & 0xFF);
        bits_out[1] = (uint8_t)((pred >> 8) & 0xFF);
        bits_out[2] = (uint8_t)idx;
        bits_out[3] = 0;
        for (int i = 0; i < ADPCM_SAMPLES; i += 2) {
            uint8_t n0 = ima_encode_nibble(pcm_in[i], &pred, &idx);
            uint8_t n1 = ima_encode_nibble(pcm_in[i + 1], &pred, &idx);
            bits_out[4 + (i >> 1)] = (uint8_t)(n0 | (n1 << 4));
        }
        s_adpcm_enc_pred = pred;
        s_adpcm_enc_index = idx;
        codec_unlock(lock);
        return ESP_OK;
    }
    if (!s_c2_enc) {
        codec_unlock(lock);
        return ESP_ERR_INVALID_STATE;
    }
    /* codec2 expects non-const but doesn't modify; cast is safe */
    codec2_encode(s_c2_enc, bits_out, (short *)pcm_in);
    codec_unlock(lock);
    return ESP_OK;
}

esp_err_t codec2_wrapper_decode(const uint8_t *bits_in, int16_t *pcm_out)
{
    SemaphoreHandle_t lock = codec_lock();
    if (s_mode == CODEC2_MODE_RAW) {
        for (int i = 0; i < RAW_SAMPLES; i++) {
            int16_t x = (int16_t)(((int32_t)bits_in[i] << 8) - 32768);
            int16_t y = (int16_t)(((int32_t)x + (int32_t)s_raw_prev_dec) >> 1);
            pcm_out[i] = y;
            s_raw_prev_dec = y;
        }
        codec_unlock(lock);
        return ESP_OK;
    }
    if (s_mode == CODEC2_MODE_ADPCM) {
        int pred = (int16_t)((uint16_t)bits_in[0] | ((uint16_t)bits_in[1] << 8));
        int idx  = clamp_index((int)bits_in[2]);
        for (int i = 0; i < ADPCM_SAMPLES; i += 2) {
            uint8_t b = bits_in[4 + (i >> 1)];
            pcm_out[i]     = ima_decode_nibble(b & 0x0F, &pred, &idx);
            pcm_out[i + 1] = ima_decode_nibble((b >> 4) & 0x0F, &pred, &idx);
        }
        codec_unlock(lock);
        return ESP_OK;
    }
    if (!s_c2_dec) {
        codec_unlock(lock);
        return ESP_ERR_INVALID_STATE;
    }
    codec2_decode(s_c2_dec, (short *)pcm_out, bits_in);
    codec_unlock(lock);
    return ESP_OK;
}

int codec2_wrapper_samples_per_frame(void)
{
    SemaphoreHandle_t lock = codec_lock();
    int n;
    if (s_mode == CODEC2_MODE_RAW) n = RAW_SAMPLES;
    else if (s_mode == CODEC2_MODE_ADPCM) n = ADPCM_SAMPLES;
    else if (!s_c2_enc) n = 160; /* default for MODE_3200 */
    else n = codec2_samples_per_frame(s_c2_enc);
    codec_unlock(lock);
    return n;
}

int codec2_wrapper_bytes_per_frame(void)
{
    SemaphoreHandle_t lock = codec_lock();
    int n;
    if (s_mode == CODEC2_MODE_RAW) n = RAW_BYTES;
    else if (s_mode == CODEC2_MODE_ADPCM) n = ADPCM_BYTES;
    else if (!s_c2_enc) n = 8;
    else n = (codec2_bits_per_frame(s_c2_enc) + 7) / 8;
    codec_unlock(lock);
    return n;
}

int codec2_wrapper_current_mode(void)
{
    SemaphoreHandle_t lock = codec_lock();
    int mode = s_mode;
    codec_unlock(lock);
    return mode;
}

void codec2_wrapper_deinit(void)
{
    SemaphoreHandle_t lock = codec_lock();
    if (s_c2_enc) { codec2_destroy(s_c2_enc); s_c2_enc = NULL; }
    if (s_c2_dec) { codec2_destroy(s_c2_dec); s_c2_dec = NULL; }
    s_raw_prev_dec = 0;
    s_adpcm_enc_pred = 0;
    s_adpcm_enc_index = 0;
    codec_unlock(lock);
}

/* =========================================================================
 * Fallback / stub path (no Codec2 library)
 * =========================================================================
 * Raw 8-bit linear PCM.
 * One CVSD PCM block = 60 samples (7.5 ms at 8 kHz) = 120 bytes PCM.
 * High-byte packing gives 2:1 → 60 bytes per block, fits in one ESP-NOW frame.
 *
 * We define a "frame" as one 7.5 ms PCM block (60 samples) so that the
 * audio pipeline works with the same block size as the HFP driver.
 * ========================================================================= */
#else /* !CONFIG_CODEC2_ENABLED */

static int s_mode = CODEC2_MODE_RAW;
static int16_t s_raw_prev_dec = 0;
static int s_adpcm_enc_pred = 0;
static int s_adpcm_enc_index = 0;

/* Stub "frame" = 60 int16_t samples (one CVSD block) */
#define STUB_SAMPLES 60
#define STUB_BYTES   60   /* PCM8 1 byte per sample */
#define STUB_ADPCM_BYTES 34

esp_err_t codec2_wrapper_init(int mode)
{
    if (!s_codec_mutex) s_codec_mutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t lock = codec_lock();
    s_mode = mode;
    s_raw_prev_dec = 0;
    s_adpcm_enc_pred = 0;
    s_adpcm_enc_index = 0;
    ESP_LOGW(TAG, "Codec2 library NOT compiled in – using raw PCM8 / ADPCM fallback");
    codec_unlock(lock);
    return ESP_OK;
}

esp_err_t codec2_wrapper_encode(const int16_t *pcm_in, uint8_t *bits_out)
{
    SemaphoreHandle_t lock = codec_lock();
    if (s_mode == CODEC2_MODE_ADPCM) {
        int pred = s_adpcm_enc_pred;
        int idx  = s_adpcm_enc_index;
        bits_out[0] = (uint8_t)(pred & 0xFF);
        bits_out[1] = (uint8_t)((pred >> 8) & 0xFF);
        bits_out[2] = (uint8_t)idx;
        bits_out[3] = 0;
        for (int i = 0; i < STUB_SAMPLES; i += 2) {
            uint8_t n0 = ima_encode_nibble(pcm_in[i], &pred, &idx);
            uint8_t n1 = ima_encode_nibble(pcm_in[i + 1], &pred, &idx);
            bits_out[4 + (i >> 1)] = (uint8_t)(n0 | (n1 << 4));
        }
        s_adpcm_enc_pred = pred;
        s_adpcm_enc_index = idx;
        codec_unlock(lock);
        return ESP_OK;
    }
    for (int i = 0; i < STUB_SAMPLES; i++) {
        bits_out[i] = (uint8_t)(((int32_t)pcm_in[i] + 32768) >> 8);
    }
    codec_unlock(lock);
    return ESP_OK;
}

esp_err_t codec2_wrapper_decode(const uint8_t *bits_in, int16_t *pcm_out)
{
    SemaphoreHandle_t lock = codec_lock();
    if (s_mode == CODEC2_MODE_ADPCM) {
        int pred = (int16_t)((uint16_t)bits_in[0] | ((uint16_t)bits_in[1] << 8));
        int idx  = clamp_index((int)bits_in[2]);
        for (int i = 0; i < STUB_SAMPLES; i += 2) {
            uint8_t b = bits_in[4 + (i >> 1)];
            pcm_out[i]     = ima_decode_nibble(b & 0x0F, &pred, &idx);
            pcm_out[i + 1] = ima_decode_nibble((b >> 4) & 0x0F, &pred, &idx);
        }
        codec_unlock(lock);
        return ESP_OK;
    }
    for (int i = 0; i < STUB_SAMPLES; i++) {
        int16_t x = (int16_t)(((int32_t)bits_in[i] << 8) - 32768);
        int16_t y = (int16_t)(((int32_t)x + (int32_t)s_raw_prev_dec) >> 1);
        pcm_out[i] = y;
        s_raw_prev_dec = y;
    }
    codec_unlock(lock);
    return ESP_OK;
}

int codec2_wrapper_samples_per_frame(void)
{
    return STUB_SAMPLES;
}

int codec2_wrapper_bytes_per_frame(void)
{
    SemaphoreHandle_t lock = codec_lock();
    int n = (s_mode == CODEC2_MODE_ADPCM) ? STUB_ADPCM_BYTES : STUB_BYTES;
    codec_unlock(lock);
    return n;
}

int codec2_wrapper_current_mode(void)
{
    SemaphoreHandle_t lock = codec_lock();
    int mode = s_mode;
    codec_unlock(lock);
    return mode;
}

void codec2_wrapper_deinit(void)
{
    SemaphoreHandle_t lock = codec_lock();
    s_raw_prev_dec = 0;
    s_adpcm_enc_pred = 0;
    s_adpcm_enc_index = 0;
    codec_unlock(lock);
}

#endif /* CONFIG_CODEC2_ENABLED */

/* =========================================================================
 * Mode-name lookup (shared between both paths)
 * ========================================================================= */
const char *codec2_wrapper_mode_name(int mode)
{
    switch (mode) {
        case CODEC2_MODE_3200: return "3200 bps";
        case CODEC2_MODE_2400: return "2400 bps";
        case CODEC2_MODE_1600: return "1600 bps";
        case CODEC2_MODE_1400: return "1400 bps";
        case CODEC2_MODE_1300: return "1300 bps";
        case CODEC2_MODE_1200: return "1200 bps";
        case CODEC2_MODE_700C: return " 700 bps";
        case CODEC2_MODE_ADPCM:return "ADPCM 32kbps";
        case CODEC2_MODE_RAW:  return "raw PCM8";
        default:               return "unknown";
    }
}
