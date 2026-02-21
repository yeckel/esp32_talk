/*
 * Display Interface – stub implementation
 *
 * Replace the bodies of these functions with SSD1306/SH1106 driver calls
 * once the hardware is connected.  See config.h for pin definitions.
 */

#include "display.h"
#include "esp_log.h"

static const char *TAG = "DISP";

/* -------------------------------------------------------------------------
 * Change this to 1 (or define CONFIG_DISPLAY_ENABLED in Kconfig) to
 * activate the real display driver.
 * ------------------------------------------------------------------------- */
#define DISPLAY_ENABLED  0

#if DISPLAY_ENABLED
/* TODO: include the SSD1306 driver header here, e.g.:
 *   #include "ssd1306.h"
 */
#endif

void display_init(void)
{
#if DISPLAY_ENABLED
    /* TODO: i2c_master_init(); ssd1306_init(); ssd1306_clear(); */
    ESP_LOGI(TAG, "OLED display initialised");
#else
    ESP_LOGD(TAG, "Display disabled (stub)");
#endif
}

void display_show_state(wt_state_t state)
{
#if DISPLAY_ENABLED
    /* TODO: ssd1306_draw_string(0, 0, walkie_talkie_state_name(state)); */
    (void)state;
#else
    (void)state;
#endif
}

void display_show_info(const char *msg)
{
#if DISPLAY_ENABLED
    /* TODO: ssd1306_draw_string(0, 5, msg); */
    (void)msg;
#else
    (void)msg;
#endif
}

void display_update_tx_activity(void)
{
#if DISPLAY_ENABLED
    /* TODO: toggle TX indicator pixel / icon */
#endif
}

void display_update_rx_activity(void)
{
#if DISPLAY_ENABLED
    /* TODO: toggle RX indicator pixel / icon */
#endif
}

void display_deinit(void)
{
#if DISPLAY_ENABLED
    /* TODO: ssd1306_off(); i2c_driver_delete(); */
    ESP_LOGI(TAG, "Display de-initialised");
#endif
}
