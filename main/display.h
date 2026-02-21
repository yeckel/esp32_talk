/*
 * Display Interface (OLED stub)
 *
 * Provides an abstraction layer for the future SSD1306 / SH1106 OLED panel.
 * All functions are no-ops until the display hardware and driver are wired in.
 *
 * To enable the display:
 *  1. Uncomment the pin defines in config.h (DISPLAY_I2C_SDA_PIN, etc.)
 *  2. Add an SSD1306 component (e.g., esp-idf-ssd1306 from the IDF registry)
 *  3. Replace the stub implementations in display.c with real calls.
 *
 * Typical SSD1306 128×64 OLED layout (walkie-talkie UI):
 *
 *  ┌────────────────────────────────┐
 *  │ WT-ESP32              [state]  │  row 0 (title + state icon)
 *  │ BT: F4:4E:FD:00:12:15         │  row 1 (headset MAC)
 *  │ Peer: FF:FF:FF:FF:FF:FF  ch:1 │  row 2 (peer MAC + channel)
 *  │ Codec: 3200 bps               │  row 3 (codec mode)
 *  │ TX: →  1234   RX: ←  5678    │  row 4 (frame counters with activity)
 *  └────────────────────────────────┘
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "walkie_talkie.h"  /* wt_state_t */

/**
 * @brief  Initialise the display hardware.
 *         No-op if DISPLAY_I2C_SDA_PIN is not defined in config.h.
 */
void display_init(void);

/**
 * @brief  Refresh the full display with the current system status.
 *         Called periodically or on state change.
 */
void display_show_state(wt_state_t state);

/**
 * @brief  Show a short informational message on line 5 for ~2 seconds.
 */
void display_show_info(const char *msg);

/**
 * @brief  Blink TX activity indicator (call each time an audio frame is sent).
 */
void display_update_tx_activity(void);

/**
 * @brief  Blink RX activity indicator (call each time an audio frame is received).
 */
void display_update_rx_activity(void);

/**
 * @brief  De-initialise and power off the display.
 */
void display_deinit(void);

#ifdef __cplusplus
}
#endif
