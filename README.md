# ESP32-based gesture recognition board

This code reads impedance, resistor and IMU values periodically and sends the readings via WiFi to the phone app nearby.

## Requirements

- Make sure you have a USB port and micro-USB cable
- Install ESP-IDF (version 4.3) standalone version or VS Code plugin.
  - General installation guide: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation-step-by-step
  - Tools Installer for Windows: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html#get-started-windows-tools-installer
  - VS Code Plugin: https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md
- Move code and build files to a separate directory. Make sure the full path contains no spaces!

## Compiling and Running

### On Linux

1. Open a Terminal
2. Export the IDF\_PATH environment variable to point to your ESP-IDF installation (usually ~/esp/esp-idf).
3. Plug in the ESP32 module.
4. Execute `./run.sh`

### On Windows

1. Launch an ESP-IDF CMD or Powershell Window.
2. Change to the project directory (`cd c:\path\to\esp\code`)
3. Build: `idf.py build`
4. Flash to ESP32: `idf.py flash -p COM5` (replace with correct COM port number). If the flashing process is stuck at "Connecting", hold down the GPIO0 button on the board.
5. Monitor using Arduino IDE or `idf.py monitor -p COM5`.

Alternatively, use the ESP-IDF plugin for Visual Studio Code.

## Code structure

- Main program logic (setup, measurement loop, calculations, printing): **main.cpp**
- General program settings: **settings.h**
- Hardware interface and pin definitions: **common.h**
- WiFi handling: **wireless.cpp**
- Component interface: classes **I2CDevice**, **AD5933**, **AD7680**, **LSM6**, **LIS2**.



