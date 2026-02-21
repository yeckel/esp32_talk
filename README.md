| Supported Targets | ESP32 |
| ----------------- | ----- |

# ESP32 Duplex Walkie-Talkie (Experimental)

This project tests full-duplex voice between two ESP32 boards:

`BT Headset ↔ ESP32 ↔ ESP-NOW ↔ ESP32 ↔ BT Headset`

In practice, this architecture is **not good for voice quality**. Running BT Classic (HFP/SCO) and ESP-NOW on the same 2.4 GHz ESP32 causes heavy coexistence pressure.

## Reality Check

- Intelligibility is generally poor.
- Best practical result is with `bw 0` (Codec2 3200 bps).
- Other `bw` modes are mainly for study/experiments.
- This repo is useful as a technical demo, not as a production intercom.

## Quick Use

### Build and flash

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Basic workflow

```text
hfp_ag> scan
hfp_ag> pair <HEADSET_MAC>
hfp_ag> setmac <HEADSET_MAC>
hfp_ag> setpeer <OTHER_ESP32_WIFI_MAC>
hfp_ag> setch 1
hfp_ag> bw 0
hfp_ag> con
```

## Important Commands

- `scan [secs]` – discover BT devices
- `pair <MAC>` / `unpair <MAC>` – manage bonding
- `setmac <MAC>` – save headset MAC
- `setpeer <MAC>` – save remote ESP32 Wi-Fi MAC
- `setch <1-13>` – ESP-NOW channel (must match both boards)
- `bw <id>` – codec mode (**use `bw 0` for best intelligibility**)
- `con` / `dis` – connect/disconnect HFP
- `status` – print current runtime status

## Notes

- Keep both ESP32 units on the same channel.
- Current default path is narrow-band CVSD (8 kHz).
- If connection gets unstable, remove stale bonds and pair again.

## Acknowledgements

Parts of this codebase were developed with help of GPT Codex and Claude Sonnet.

## License

Copyright (C) 2024-2026 Libor Tomsik

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License v3.0 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See [LICENSE](LICENSE) for details.
