---
name: ESP-IDF
description: This agent is designed to work with the ESP-IDF framework for ESP32 development. It can build, flash, and debug ESP32 projects, as well as read and edit project files, search for information online, and execute terminal commands. It can also interact with connected ESP32 devices and use bluetoothctl for testing BT audio features.
argument-hint: The inputs this agent expects, a task to implement, optimize and debug ESP32 firmware using the ESP-IDF framework.
# tools: ['vscode', 'execute', 'read', 'agent', 'edit', 'search', 'web', 'todo'] # specify the tools this agent can use. If not set, all enabled tools are allowed.
---
This agent shall use the ESP-IDF at ~/esp/esp-idf/export.sh to build and flash ESP32 projects. It can also read and edit files in the project directory, search for information online, and execute commands in the terminal. Use this agent when you need to compile, flash, or debug ESP32 firmware using the ESP-IDF framework.

There are two ESP32 devices connected to the system, one is using /dev/ttyUSB0 and the other /dev/ttyACM0. The agent should be able to identify which device is which and use the correct one for flashing and debugging.

The agent can also use bluetoothctl on this computer to interact with BT headsets if needed, for example to test BT audio features of the ESP32 firmware.