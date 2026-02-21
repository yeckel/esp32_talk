/*
 * ESP32 Duplex Walkie-Talkie Configuration
 *
 * Edit this file to set your device's MAC addresses, ESP-NOW channel,
 * and Codec2 audio bandwidth before building.
 *
 * Device workflow:
 *   Device A (headset MAC set)  <--ESP-NOW--> Device B (headset MAC set)
 *   Both ESP32 boards act as HFP Audio Gateways for their respective headsets.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * BLUETOOTH HEADSET
 * ========================================================================= */

/** Default Bluetooth MAC address of the headset.
 *  Overridable at runtime with:  setmac XX:XX:XX:XX:XX:XX
 *  Persisted to NVS after the first successful pairing / setmac command.
 */
#define DEFAULT_HEADSET_MAC   { 0xF4, 0x4E, 0xFD, 0x00, 0x12, 0x15 }

/** Bluetooth device name advertised by this ESP32. */
#define BT_DEVICE_NAME        "WT-ESP32"

/* =========================================================================
 * ESP-NOW PEER (the other ESP32 walkie-talkie unit)
 * ========================================================================= */

/** MAC address of the remote ESP32 unit (peer in ESP-NOW).
 *  Use 'setpeer XX:XX:XX:XX:XX:XX' to update at runtime.
 *  Set to broadcast {0xFF,...} to send to any listening unit (useful for
 *  initial commissioning / testing).
 */
#define DEFAULT_ESPNOW_PEER_MAC  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

/** Wi-Fi channel for ESP-NOW (1-13).
 *  Both units MUST be on the same channel.
 *  Use 'setch <N>' to update at runtime.
 */
#define DEFAULT_ESPNOW_CHANNEL   1

/* =========================================================================
 * CODEC2 AUDIO CODEC
 * ========================================================================= */

/** Codec2 mode IDs – must match the codec2.h defines:
 *   CODEC2_MODE_3200  = 0   (3200 bps, 20 ms frames, 160 samples, 8 bytes)
 *   CODEC2_MODE_2400  = 1   (2400 bps, 20 ms frames, 160 samples, 6 bytes)
 *   CODEC2_MODE_1600  = 2   (1600 bps, 40 ms frames, 320 samples, 8 bytes)
 *   CODEC2_MODE_1400  = 3   (1400 bps, 40 ms frames, 320 samples, 7 bytes)
 *   CODEC2_MODE_1300  = 4   (1300 bps, 40 ms frames, 320 samples, 7 bytes)
 *   CODEC2_MODE_1200  = 5   (1200 bps, 40 ms frames, 320 samples, 6 bytes)
 *   CODEC2_MODE_700C  = 8   ( 700 bps, 40 ms frames, 320 samples, 4 bytes)
 *   CODEC2_MODE_ADPCM = 9   (IMA ADPCM, 7.5 ms frames, 60 samples, 34 bytes)
 *
 * When the codec2 component is NOT present, the firmware falls back to raw
 * PCM passthrough (CVSD, 120 bytes / 7.5 ms → one ESP-NOW packet per block).
 */
#define DEFAULT_CODEC2_MODE      0  /* CODEC2_MODE_3200 */

/* =========================================================================
 * NVS STORAGE KEYS
 * ========================================================================= */

#define NVS_NAMESPACE            "wt_cfg"
#define NVS_KEY_HEADSET_MAC      "hs_mac"
#define NVS_KEY_PEER_MAC         "peer_mac"
#define NVS_KEY_ESPNOW_CHANNEL   "en_chan"
#define NVS_KEY_CODEC2_MODE      "c2_mode"
#define NVS_KEY_VOX_THRESHOLD    "vox_thr"

/* =========================================================================
 * AUDIO / PCM PARAMETERS
 * ========================================================================= */

/** HFP PCM block duration used by the ESP-IDF HFP driver (microseconds). */
#define PCM_BLOCK_DURATION_US    7500U

/** Narrow-band CVSD sample rate (kHz). */
#define PCM_SAMPLE_RATE_KHZ      8U

/** Wideband mSBC sample rate (kHz). */
#define PCM_WBS_SAMPLE_RATE_KHZ  16U

/** Bytes per PCM sample. */
#define BYTES_PER_SAMPLE         2U

/** PCM bytes per CVSD block (=60 samples * 2 bytes = 120). */
#define CVSD_BLOCK_BYTES  (PCM_SAMPLE_RATE_KHZ * PCM_BLOCK_DURATION_US / 1000U * BYTES_PER_SAMPLE)

/** PCM bytes per WBS block (=120 samples * 2 bytes = 240). */
#define WBS_BLOCK_BYTES  (PCM_WBS_SAMPLE_RATE_KHZ * PCM_BLOCK_DURATION_US / 1000U * BYTES_PER_SAMPLE)

/** Number of encoded frames to batch into one ESP-NOW packet.
 *  2 frames = 40 ms of audio per packet @ 50 frames/sec → 25 packets/sec.
 *  Lower values reduce latency and the impact of packet loss.
 *  Keep ≤ ESPNOW_MAX_PAYLOAD / bytes_per_frame. */
#define WT_TX_BATCH_FRAMES   2U

/** Batch size for high-rate modes (RAW PCM8 and ADPCM).
 *  4 frames lowers packet rate (better BT/Wi-Fi coexistence) at the cost
 *  of extra latency. */
#define WT_TX_BATCH_FRAMES_HI_RATE 4U

/* =========================================================================
 * VOX (Voice-Activated Transmission)
 * ========================================================================= */

/** Default VOX threshold (RMS of 160 int16_t samples).
 *  0 = VOX disabled (always transmit).  Typical speech: 500-5000.
 *  Background noise floor is usually < 100-200. */
#define DEFAULT_VOX_THRESHOLD   200U

/** VOX hang-time in Codec2 frames.  After voice drops below threshold,
 *  keep transmitting this many more frames to avoid clipping word tails.
 *  At 3200 bps (20 ms/frame): 15 frames ≈ 300 ms. */
#define VOX_HANGTIME_FRAMES     15U

/** Codec2 TX PCM ring-buffer size (bytes). Holds ~10 WBS frames of headroom. */
#define WT_TX_RINGBUF_SIZE   4800U

/** Decoded-PCM RX ring-buffer size fed to the HFP outgoing callback.
 *  Sized for WBS with deeper headroom to absorb BT/Wi-Fi jitter bursts. */
#define WT_RX_RINGBUF_SIZE   12000U

/** Jitter buffer pre-roll in HFP audio blocks before starting playout.
 *  One block is typically 7.5 ms (120 bytes NBS, 240 bytes WBS).
 *  16 blocks ≈ 120 ms startup delay for smoother playout. */
#define WT_JB_PREROLL_BLOCKS 16U

/** When decoded PCM queued drops below this many blocks, re-enter pre-roll.
 *  Keep below WT_JB_PREROLL_BLOCKS to avoid oscillation. */
#define WT_JB_REBUFFER_BLOCKS 8U

/* =========================================================================
 * ESP-NOW PACKET FRAMING
 * ========================================================================= */

/** Magic byte that identifies a valid ESP-NOW audio packet. */
#define ESPNOW_AUDIO_MAGIC   0xABU

/** Maximum ESP-NOW payload (ESP-IDF limit is 250 bytes). */
#define ESPNOW_MAX_PAYLOAD   245U

/* =========================================================================
 * DISPLAY (future OLED)
 * ========================================================================= */

/** Uncomment and set I2C pins when the OLED display is connected. */
// #define DISPLAY_I2C_SDA_PIN   21
// #define DISPLAY_I2C_SCL_PIN   22
// #define DISPLAY_I2C_ADDR      0x3C

#ifdef __cplusplus
}
#endif
