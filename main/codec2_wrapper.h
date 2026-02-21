/*
 * Codec2 Wrapper for ESP32 Walkie-Talkie
 *
 * Provides a simple encode/decode interface for Codec2 audio compression.
 *
 * When the `codec2` ESP-IDF component is present (see components/codec2/)
 * the real Codec2 library is used.  Without it the wrapper falls back to
 * raw 8-bit PCM passthrough so the firmware still compiles and runs.
 *
 * Supported Codec2 modes (bandwidth    → bits per frame / bytes per frame):
 *   CODEC2_MODE_3200  (3200 bps, 20 ms / 160 samples →  64 bits /  8 bytes)
 *   CODEC2_MODE_2400  (2400 bps, 20 ms / 160 samples →  48 bits /  6 bytes)
 *   CODEC2_MODE_1600  (1600 bps, 40 ms / 320 samples →  64 bits /  8 bytes)
 *   CODEC2_MODE_1400  (1400 bps, 40 ms / 320 samples →  56 bits /  7 bytes)
 *   CODEC2_MODE_1300  (1300 bps, 40 ms / 320 samples →  52 bits /  7 bytes)
 *   CODEC2_MODE_1200  (1200 bps, 40 ms / 320 samples →  48 bits /  6 bytes)
 *   CODEC2_MODE_700C  ( 700 bps, 40 ms / 320 samples →  28 bits /  4 bytes)
 *   CODEC2_MODE_ADPCM (IMA ADPCM, 7.5 ms / 60 samples → 34 bytes incl. state)
 *
 * Raw fallback:
 *   CODEC2_MODE_RAW (0xFF) – identity: 8-bit linear PCM (high-byte packed),
 *   same frame cadence as CVSD PCM blocks.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"

/* Mode ID constants – identical to codec2.h values */
#define CODEC2_MODE_3200   0
#define CODEC2_MODE_2400   1
#define CODEC2_MODE_1600   2
#define CODEC2_MODE_1400   3
#define CODEC2_MODE_1300   4
#define CODEC2_MODE_1200   5
#define CODEC2_MODE_700C   8
#define CODEC2_MODE_ADPCM  9
#define CODEC2_MODE_RAW    0xFF  /* fallback passthrough */

/**
 * @brief Initialise (or reinitialise) the Codec2 encoder and decoder.
 *
 * Safe to call multiple times; re-creates the codec2 state on mode change.
 *
 * @param mode  One of the CODEC2_MODE_* defines above.
 * @return      ESP_OK on success.
 */
esp_err_t codec2_wrapper_init(int mode);

/**
 * @brief Encode one Codec2 frame worth of PCM samples.
 *
 * @param[in]  pcm_in   Pointer to codec2_wrapper_samples_per_frame() int16_t samples.
 *                      Samples are 8-kHz, 16-bit signed, mono.
 * @param[out] bits_out Pointer to a buffer of at least codec2_wrapper_bytes_per_frame()
 *                      bytes that will receive the encoded frame.
 * @return     ESP_OK always (encode cannot fail once initialised).
 */
esp_err_t codec2_wrapper_encode(const int16_t *pcm_in, uint8_t *bits_out);

/**
 * @brief Decode one Codec2 frame into PCM samples.
 *
 * @param[in]  bits_in   Codec2-encoded frame (codec2_wrapper_bytes_per_frame() bytes).
 * @param[out] pcm_out   Buffer for codec2_wrapper_samples_per_frame() int16_t samples.
 * @return     ESP_OK always.
 */
esp_err_t codec2_wrapper_decode(const uint8_t *bits_in, int16_t *pcm_out);

/**
 * @brief Number of int16_t PCM samples per Codec2 frame (160 or 320).
 *        Must have this many samples before calling codec2_wrapper_encode().
 */
int codec2_wrapper_samples_per_frame(void);

/**
 * @brief Number of encoded bytes produced / consumed per Codec2 frame.
 */
int codec2_wrapper_bytes_per_frame(void);

/**
 * @brief Currently active Codec2 mode.
 */
int codec2_wrapper_current_mode(void);

/**
 * @brief Human-readable name for a mode id (e.g. "3200 bps").
 */
const char *codec2_wrapper_mode_name(int mode);

/**
 * @brief Release Codec2 resources.
 */
void codec2_wrapper_deinit(void);

#ifdef __cplusplus
}
#endif
