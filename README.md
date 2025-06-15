# PlutoPAD-OS Firmware

**C++ based firmware for the PlutoPAD wireless controller, handling button input, battery charging/monitoring, and Bluetooth Serial communication.**

PlutoPAD-OS is the embedded firmware that powers the PlutoPAD controller, based on a ESP32 microcontroller chip and designed for wireless robotics projects. 
PlutoPAD-OS firmware reads the state of 16 buttons using an I²C-connected MCP23017 GPIO expander and transmits input events over Bluetooth using the Serial Port 
Profile (SPP) for fast and reliable communication. The firmware integrates real-time battery monitoring via the INA226 sensor, allowing it to track 
voltage and current draw, and automatically enters deep sleep mode when inactive to preserve battery life during idle periods.

---
## 🔧 Board Features

- Microcontroller :  ESP32-WROOM-32E (Bluetooth + WiFi | 3.3V System)
- GPIO Expander   :  MCP23017 (via I²C)
- LiPo Monitor    :  INA226
- LiPo Charger    :  MCP73831 (@500mA MAX)
- 3.3V Regulator  :  TPS73733
---

## 📦 Dependencies

Before building PlutoPAD-OS, make sure the following libraries are installed in your IDE or firmware environment:

- **BluetoothSerial**  
  Included with the ESP32 board support package. No separate installation required.  

- **Adafruit MCP23X17**  
  Used to interface with the I²C GPIO expander (MCP23017).  
  [Adafruit MCP23017 Library](https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library)  

- **INA226** (by Rob Tillaart)  
  Used for battery voltage and current monitoring over I²C.  
  [INA226 by Rob Tillaart](https://github.com/RobTillaart/INA226)  

- **esp_sleep**  
  Built into the ESP32 core; used for controlling deep sleep and power management. No separate installation required.  

### Arduino IDE

To install libraries via Arduino IDE:

1. Go to **Sketch > Include Library > Manage Libraries…**
2. Search for each library name (e.g. *INA226* by Rob Tillaart)
3. Click **Install**

### PlatformIO

For PlatformIO users, add the following to your `platformio.ini`:

```ini
lib_deps =
  adafruit/Adafruit MCP23017 Arduino Library
  robtillaart/INA226
```
---

## 📄 License
**This library is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html).**

You are free to use, modify, and distribute this library under the terms of the GPL.
