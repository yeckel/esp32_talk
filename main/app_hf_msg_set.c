/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_hf_ag_api.h"
#include "esp_gap_bt_api.h"
#include "app_hf_msg_set.h"
#include "bt_app_hf.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "walkie_talkie.h"
#include "codec2_wrapper.h"

/* shared peer address – kept in sync with walkie_talkie config */
esp_bd_addr_t hf_peer_addr = {0xF4, 0x4E, 0xFD, 0x00, 0x12, 0x15};

static bool parse_bt_mac(const char *str, uint8_t *mac)
{
    int v[6];
    if (sscanf(str, "%x:%x:%x:%x:%x:%x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
    return true;
}

/* =========================================================================
 * BT connection commands
 * ========================================================================= */

static int cmd_con(int argc, char **argv)
{
    wt_config_t cfg;
    walkie_talkie_get_config(&cfg);
    if (argc >= 2) {
        if (!parse_bt_mac(argv[1], cfg.headset_mac)) {
            printf("Invalid MAC. Use format XX:XX:XX:XX:XX:XX\n");
            return 1;
        }
        walkie_talkie_set_headset_mac(cfg.headset_mac);
    }
    memcpy(hf_peer_addr, cfg.headset_mac, 6);
    printf("Connecting to %02X:%02X:%02X:%02X:%02X:%02X ...\n",
           hf_peer_addr[0], hf_peer_addr[1], hf_peer_addr[2],
           hf_peer_addr[3], hf_peer_addr[4], hf_peer_addr[5]);
    esp_hf_ag_slc_connect(hf_peer_addr);
    return 0;
}

static int cmd_dis(int argc, char **argv)
{
    printf("Disconnecting...\n");
    esp_hf_ag_slc_disconnect(hf_peer_addr);
    return 0;
}

static int cmd_call(int argc, char **argv)
{
    printf("Starting audio...\n");
    esp_hf_ag_audio_connect(hf_peer_addr);
    return 0;
}

static int cmd_endcall(int argc, char **argv)
{
    printf("Ending audio...\n");
    esp_hf_ag_audio_disconnect(hf_peer_addr);
    return 0;
}

static int cmd_vol(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: vol <target> <level>\n");
        printf("  target: 0=speaker  1=microphone\n");
        printf("  level:  0-15\n");
        return 1;
    }
    int target = atoi(argv[1]);
    int volume = atoi(argv[2]);
    if (target != ESP_HF_VOLUME_CONTROL_TARGET_SPK &&
        target != ESP_HF_VOLUME_CONTROL_TARGET_MIC) {
        printf("Invalid target (use 0 or 1).\n");
        return 1;
    }
    if (volume < 0 || volume > 15) {
        printf("Volume must be 0-15.\n");
        return 1;
    }
    esp_hf_ag_volume_control(hf_peer_addr, target, volume);
    printf("Volume set: target=%d level=%d\n", target, volume);
    return 0;
}

//+CIEV
/* =========================================================================
 * Walkie-talkie configuration / status commands
 * ========================================================================= */

static int cmd_setmac(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: setmac <MAC>   e.g. setmac F4:4E:FD:00:12:15\n");
        return 1;
    }
    uint8_t mac[6];
    if (!parse_bt_mac(argv[1], mac)) {
        printf("Invalid MAC address format. Use XX:XX:XX:XX:XX:XX\n");
        return 1;
    }
    esp_err_t ret = walkie_talkie_set_headset_mac(mac);
    if (ret == ESP_OK) {
        memcpy(hf_peer_addr, mac, 6);
        printf("Headset MAC saved: %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        printf("NVS save failed: %s\n", esp_err_to_name(ret));
    }
    return 0;
}

static int cmd_setpeer(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: setpeer <MAC>  (MAC of the remote ESP32 unit)\n");
        printf("       Use FF:FF:FF:FF:FF:FF for broadcast\n");
        return 1;
    }
    uint8_t mac[6];
    if (!parse_bt_mac(argv[1], mac)) {
        printf("Invalid MAC address.\n");
        return 1;
    }
    esp_err_t ret = walkie_talkie_set_peer_mac(mac);
    if (ret == ESP_OK)
        printf("ESP-NOW peer set to %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    else
        printf("Failed: %s\n", esp_err_to_name(ret));
    return 0;
}

static int cmd_setch(int argc, char **argv)
{
    if (argc != 2) { printf("Usage: setch <1-13>\n"); return 1; }
    int ch = atoi(argv[1]);
    if (ch < 1 || ch > 13) { printf("Channel must be 1-13.\n"); return 1; }
    esp_err_t ret = walkie_talkie_set_channel((uint8_t)ch);
    if (ret == ESP_OK) printf("ESP-NOW channel set to %d.\n", ch);
    else              printf("Failed: %s\n", esp_err_to_name(ret));
    return 0;
}

static int cmd_bw(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: bw <mode>   Codec2 bandwidth mode:\n");
        printf("  0=3200bps  1=2400bps  2=1600bps  3=1400bps\n");
        printf("  4=1300bps  5=1200bps  8=700bps  9=ADPCM  255=raw-PCM8\n");
        return 1;
    }
    int mode = atoi(argv[1]);
    if (!(mode == 0 || mode == 1 || mode == 2 || mode == 3 || mode == 4 ||
          mode == 5 || mode == 8 || mode == 9 || mode == 255)) {
        printf("Unsupported mode. Use: 0,1,2,3,4,5,8,9,255\n");
        return 1;
    }
    esp_err_t ret = walkie_talkie_set_codec2_mode(mode);
    if (ret == ESP_OK)
        printf("Codec2 mode set to %d (%s)\n", mode, codec2_wrapper_mode_name(mode));
    else
        printf("Failed: %s\n", esp_err_to_name(ret));
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    walkie_talkie_print_status();
    return 0;
}

static int cmd_macs(int argc, char **argv)
{
    uint8_t wifi_sta[6] = {0};
    uint8_t bt[6] = {0};

    esp_err_t ret_wifi = esp_read_mac(wifi_sta, ESP_MAC_WIFI_STA);
    esp_err_t ret_bt = esp_read_mac(bt, ESP_MAC_BT);

    if (ret_wifi == ESP_OK) {
        printf("Local Wi-Fi STA MAC : %02X:%02X:%02X:%02X:%02X:%02X\n",
               wifi_sta[0], wifi_sta[1], wifi_sta[2],
               wifi_sta[3], wifi_sta[4], wifi_sta[5]);
    } else {
        printf("Local Wi-Fi STA MAC : <error: %s>\n", esp_err_to_name(ret_wifi));
    }

    if (ret_bt == ESP_OK) {
        printf("Local BT MAC       : %02X:%02X:%02X:%02X:%02X:%02X\n",
               bt[0], bt[1], bt[2], bt[3], bt[4], bt[5]);
    } else {
        printf("Local BT MAC       : <error: %s>\n", esp_err_to_name(ret_bt));
    }

    return 0;
}

static int cmd_resetstats(int argc, char **argv)
{
    walkie_talkie_reset_stats();
    printf("Traffic stats reset.\n");
    return 0;
}

static int cmd_scan(int argc, char **argv)
{
    int dur = (argc > 1) ? atoi(argv[1]) : 10;
    if (dur < 1 || dur > 48) { printf("Duration must be 1-48 seconds.\n"); return 1; }
    printf("Scanning for %d seconds...\n", dur);
    esp_err_t ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, dur, 0);
    if (ret != ESP_OK) { printf("Error: %s\n", esp_err_to_name(ret)); return 1; }
    return 0;
}

static int cmd_stopscan(int argc, char **argv)
{
    esp_bt_gap_cancel_discovery();
    printf("Discovery stopped.\n");
    return 0;
}

static int cmd_paired(int argc, char **argv)
{
    int n = esp_bt_gap_get_bond_device_num();
    if (n == 0) { printf("No paired devices.\n"); return 0; }
    esp_bd_addr_t *list = malloc(sizeof(esp_bd_addr_t) * n);
    if (!list) { printf("Out of memory.\n"); return 1; }
    esp_bt_gap_get_bond_device_list(&n, list);
    printf("Paired devices (%d):\n", n);
    for (int i = 0; i < n; i++)
        printf("  %d. %02X:%02X:%02X:%02X:%02X:%02X\n", i + 1,
               list[i][0], list[i][1], list[i][2],
               list[i][3], list[i][4], list[i][5]);
    free(list);
    return 0;
}

static int cmd_pair(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: pair <MAC>   (e.g. pair F4:4E:FD:00:12:15)\n");
        return 1;
    }
    esp_bd_addr_t bda;
    if (!parse_bt_mac(argv[1], bda)) { printf("Invalid MAC.\n"); return 1; }
    printf("Pairing with %s ...\n", argv[1]);
    esp_bt_gap_cancel_discovery();
    esp_err_t ret = esp_bt_gap_get_remote_services(bda);
    if (ret != ESP_OK) { printf("Pair failed: %s\n", esp_err_to_name(ret)); return 1; }
    printf("Pairing initiated.\n");
    return 0;
}

static int cmd_unpair(int argc, char **argv)
{
    if (argc != 2) { printf("Usage: unpair <MAC>\n"); return 1; }
    esp_bd_addr_t bda;
    if (!parse_bt_mac(argv[1], bda)) { printf("Invalid MAC.\n"); return 1; }
    esp_err_t ret = esp_bt_gap_remove_bond_device(bda);
    printf(ret == ESP_OK ? "Device unpaired.\n" : "Unpair failed: %s\n", esp_err_to_name(ret));
    return 0;
}

static int cmd_vox(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: vox <threshold>\n");
        printf("  0     = VOX off (PTT only)\n");
        printf("  100   = very sensitive (quiet room)\n");
        printf("  200   = default\n");
        printf("  500+  = noisy environment\n");
        return 1;
    }
    int val = atoi(argv[1]);
    if (val < 0 || val > 30000) { printf("Range: 0-30000\n"); return 1; }
    walkie_talkie_set_vox_threshold((uint16_t)val);
    printf("VOX threshold set to %d%s\n", val, val == 0 ? " (disabled)" : "");
    return 0;
}

static int cmd_ptt(int argc, char **argv)
{
    walkie_talkie_toggle_ptt();
    printf("PTT %s\n", walkie_talkie_get_ptt() ? "ON – transmitting" : "OFF");
    return 0;
}

#define REG(s, fn, h) do { \
    const esp_console_cmd_t _c = {.command=(s),.help=(h),.hint=NULL,.func=(fn)}; \
    ESP_ERROR_CHECK(esp_console_cmd_register(&_c)); } while(0)

void register_hfp_ag(void)
{
    /* BT connection */
    REG("con",      cmd_con,     "Connect to headset (con [MAC])");
    REG("dis",      cmd_dis,     "Disconnect headset");
    REG("call",     cmd_call,    "Open audio channel to headset");
    REG("cona",     cmd_call,    "Alias for 'call'");
    REG("endcall",  cmd_endcall, "Close audio channel");
    REG("disa",     cmd_endcall, "Alias for 'endcall' – disconnect audio");
    REG("vol",      cmd_vol,     "Volume: vol <0=spk|1=mic> <0-15>");
    REG("vu",       cmd_vol,     "Alias for 'vol'");

    /* Device management */
    REG("scan",     cmd_scan,    "Scan for BT devices (scan [seconds])");
    REG("stopscan", cmd_stopscan,"Stop BT scan");
    REG("paired",   cmd_paired,  "List bonded BT devices");
    REG("pair",     cmd_pair,    "Pair with BT device: pair <MAC>");
    REG("unpair",   cmd_unpair,  "Remove BT pairing: unpair <MAC>");

    /* Walkie-talkie config */
    REG("setmac",   cmd_setmac,  "Set & save headset MAC: setmac XX:XX:XX:XX:XX:XX");
    REG("setpeer",  cmd_setpeer, "Set & save ESP-NOW peer MAC: setpeer XX:XX:XX:XX:XX:XX");
    REG("setch",    cmd_setch,   "Set ESP-NOW Wi-Fi channel: setch <1-13>");
    REG("bw",       cmd_bw,      "Set codec mode: bw <0..5|8|9=ADPCM|255=raw>");
    REG("vox",      cmd_vox,     "Set VOX threshold: vox <0=PTT only|100-500|default 200>");
    REG("ptt",      cmd_ptt,     "Toggle push-to-talk (also: headset play/pause button)");
    REG("status",   cmd_status,  "Print full system status");
    REG("macs",     cmd_macs,    "Print local ESP32 Wi-Fi/BT MAC addresses");
    REG("resetstats", cmd_resetstats, "Reset walkie-talkie traffic counters");

    /* Sync hf_peer_addr from saved NVS config */
    wt_config_t cfg;
    walkie_talkie_get_config(&cfg);
    memcpy(hf_peer_addr, cfg.headset_mac, 6);
}



