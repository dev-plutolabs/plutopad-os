/**
 * @file PlutoPAD-OS.cpp
 * @brief Firmware code written for the PlutoPAD wireless controller
 *
 * This firmware contains the implementation of the PlutoPAD controller, which features the following ICs:
 *   - Microcontroller :  ESP32-WROOM-32E (Bluetooth + WiFi | 3.3V System)
 *   - GPIO Expander   :  MCP23017 (via I2C)
 *   - LiPo Monitor    :  INA226
 *   - LiPo Charger    :  MCP73831 (@ 500mA max)
 *   - 3.3V Regulator  :  TPS73733
 * 
 * All [x16] digital button states sent over custom 2-byte <packet> to any Bluetooth client (using PlutoPAD.h)
 * DEBUG-MODE : Hold down [CENTRE A + CENTRE B + R1] @startup | Disables Bluetooth for testing
 * WIRED-MODE : Hold down [MIDDLE A + MIDDLE B]      @startup | Disables wireless connectivity
 * 
 * Status LED [YELLOW] to indicate different system status and modes:
 *   - OFF            : No Activity
 *   - PULSE          : Bluetooth Searching...
 *   - SOLID          : Bluetooth Connected!
 *   - FAST BLINK     : Battery Low (between 3.5V and 3.3V)
 *   - FAST-STOP-FAST : Battery Critical (less than 3.3V  CHARGE NOW)
 *   - DOUBLE BLINK   : Wired Mode
 *   - TRIPLE BLINK   : Debug Mode
 * 
 * PlutoPAD is built to be a wireless controller for microcontroller-based robots and applications,
 * enabling the control of hardware through simple wireless packet transmission.
 * 
 * *
 * @author  
 * Peter Kyriakides
 *
 * @version 0.1.0
 * @date    15-06-2025
 * 
 * @note Subject to the GNU General Public License v3.0 (GPL-3.0).
 */

#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <Adafruit_MCP23X17.h>
#include <INA226.h>
#include <esp_sleep.h>

#define INA226_ADDR 0x40    // I2C address: INA226    [Battery Voltage Monitor]
#define MCP23017_ADDR 0x21  // I2C address: MCP23017  [GPIO-EXPANDER / Button Controller]

#define NUM_BUTTONS 16
#define MCU_INT 21          // Interrupt pin for MCU on the ESP32 [INT-A]

#define ESP_SCL 25
#define ESP_SDA 26

#define CRITICAL_VOLTAGE 3.3    // Critical battery voltage threshold [V]
#define LOW_VOLTAGE 3.5         // Low battery voltage threshold [V]
#define RECOVERY_VOLTAGE 3.7    // Recovery/wake battery voltage threshold [V]
#define WAKE_INTERVAL 20        // Wake interval from deep sleep battery check [s]

// Define the pin for the Status LED [YELLOW]
int statusLED = 12;

// Declare Bluetooth Variables
BluetoothSerial serialBluetooth;              // Declare Bluetooth Serial instance
const char* deviceName = "PlutoPAD";          // Bluetooth ID of <<THIS>> device
bool client_isConnected = false;              // Flag to check if a Bluetooth Client is connected
const unsigned long modeSelectTimeout = 2000; // Debug mode activation timeout [ms] 
bool enableBluetooth = true;                  // Flag to disable Bluetooth connection for non-wireless mode [DEFAULT: ENABLED]
bool enableDebug = true;                      // Flag to enable debug mode [DEFAULT: ENABLED]
bool enableWired = false;                     // Flag to enable wire mode for game controller [DEFAULT: DISABLED]

// Declare battery monitor variables
unsigned long prevBatteryCheck = 0;       // Previous time for battery voltage check [ms]
const long batteryCheckInterval = 5000;   // Interval for battery voltage check [ms]
bool startupBatteryCheck = true;          // Flag to check battery voltage on power-on / wakeup

// Define button controller
Adafruit_MCP23X17 buttonController;

// Define battery voltage monitor
INA226 INA(INA226_ADDR);

// Define a struct for PlutoPAD button data
struct Button 
{
  const uint8_t CHIP_POSITION; // Button position on MCP23017 chip
  const char* name; // Button name on controller
  bool state; // Button current state (pressed/released)
};

// Define a struct for the status LED mode
enum LEDMode 
{
  LED_OFF,            // LED OFF:           No Activity
  LED_PULSE,          // LED PULSE:         Bluetooth Searching (Mode 1)
  LED_SOLID,          // LED SOLID:         Bluetooth Connected (Mode 1)
  LED_SLOW_BLINK,     // LED SLOW BLINK:    [unassigned]
  LED_FAST_BLINK,     // LED FAST BLINK:    CAUTION: Battery Low (3.3V <= VBAT <= 3.5V)
  LED_DOUBLE_BLINK,   // LED DOUBLE BLINK:  Wired Mode (Mode 2)
  LED_TRIPLE_BLINK,   // LED TRIPLE BLINK:  Debug Mode (Mode 3)
};
LEDMode currentLEDMode = LED_OFF;       // Default LED mode
LEDMode batteryLEDOverride = LED_OFF;   // Override LED mode for battery status

// Define all PlutoPAD button pins and names
// - Order defined by connection to MCP23017 chip (GPIOA: 0-7, GPIOB: 0-7 [8-15])
Button buttons[] = 
{
  {0, "CENTRE_B", false}, {1, "DOWN_B", false}, {2, "LEFT_B", false}, {3, "MIDDLE_B", false},
  {4, "RIGHT_B",false}, {5, "UP_B", false}, {6, "R1", false}, {7, "R2", false},
  {8, "L2", false}, {9, "L1", false}, {10, "UP_A", false}, {11, "RIGHT_A", false},
  {12, "MIDDLE_A", false}, {13, "LEFT_A", false}, {14, "DOWN_A", false}, {15, "CENTRE_A", false}
};

// Declare function signatures
void checkBluetoothClient(void);
void checkMode(void);
void initButtonController(void);
void readWriteButtons(void);
void enableWiredMode(void);
uint16_t getBit(uint16_t, int);
void initStatusLED(void);
void setStatusLED(uint8_t);
void updateStatusLED(void);
void handleBlinkPattern(uint8_t, uint16_t, uint16_t);
void initBatteryMonitor(void);
float readBatteryVoltage(void);
void checkBatteryVoltage(void);
bool isBatteryConnected(float); 
void lowPowerMode(void);

// Setup function to initialise functionality
void setup() 
{
  // Initialise Serial and I2C Communication
  Serial.begin(115200);
  Wire.begin(ESP_SDA, ESP_SCL);
  Serial.println("@PlutoPAD: System starting up!");

  // Initialise all PlutoPAD electrical components
  initBatteryMonitor();     // Initialise Battery Monitor
  checkBatteryVoltage();    // Check battery voltage
  initButtonController();   // Initialise Button Controller
  initStatusLED();          // Initialise Status LED pin
  checkMode();              // Check what [MODE] is enabled

  // Initialise Bluetooth serial communication normally [NOT in Debug Mode]
  if (enableBluetooth) 
  {
    if (!serialBluetooth.begin(deviceName)) // Set and Begin PlutoPAD Bluetooth ID
    {
      Serial.println("@PlutoPAD: *ERROR* Bluetooth Uninitialised ... Fix and Reboot!");
      while (1);
    } 
    else
    {
      Serial.println("@PlutoPAD: Bluetooth starting!");
      Serial.println("@PlutoPAD: Waiting to connect...");
      currentLEDMode = LED_PULSE; // Start pulsing to indicate searching for client
    }
  }
}

// Main loop perform functionality
void loop() 
{
  readWriteButtons(); // Read and write button states from MCP23017 chip

  if (enableBluetooth) 
  {
    checkBluetoothClient(); // Check Bluetooth client status
  }

  checkBatteryVoltage(); // Check battery voltage and update LED mode
  updateStatusLED(); // Update Status LED based on current mode
  
}

// Function to check if what mode is enabled on startup
//  - [DEBUG MODE] activated by pressing [CENTRE_A + CENTRE_B + R1]
//  - [WIRED MODE] activated by pressing [MIDDLE_B + MIDDLE_A]
void checkMode(void)
{
  // Check for debug mode activation within [modeSelectTimeout] second of startup
  unsigned long startTime = millis();
  Serial.println("@PlutoPAD: Checking for [MODE] activation...");
  while (millis() - startTime < modeSelectTimeout) 
  {
    readWriteButtons(); // Read button states from MCP23017 chip

    // MODE 2: Check for game controller wired mode activation
    if(buttons[3].state && buttons[12].state) // Hold: MIDDLE_B, MIDDLE_A
    {
      enableBluetooth = false;
      enableWired = true;
      currentLEDMode = LED_DOUBLE_BLINK;
      enableWiredMode();
      break;
    }

    // MODE 3: Check for debug mode activation
    if (buttons[15].state && buttons[0].state && buttons[6].state) // Hold: CENTRE_A, CENTRE_B, R1
    {
      enableBluetooth = false;
      enableDebug = true;
      currentLEDMode = LED_TRIPLE_BLINK;
      Serial.println("@PlutoPAD: [DEBUG] mode activated!");
      Serial.println("@PlutoPAD:  * Bluetooth connection disabled! *");
      break;
    }
  }
}

 // Function to initialise all buttons from MCP23017 chip
void initButtonController(void)
{
  // Initialise BUTTON controller (MCP23017)
  if (!buttonController.begin_I2C(MCP23017_ADDR)) 
  {
    Serial.println("@PlutoPAD: *ERROR* Button Controller Uninitialised!");
    while (1);
  }

  // Configure interrupt control on IC
  // Set both A and B bank to interrupt on any change
  buttonController.setupInterrupts(true, false, LOW); // Mirroring INTA & INTB as override, open-drain output

  // Initialise each button as input (internal pullups)
  for (int i = 0; i < NUM_BUTTONS; i++) 
  {
    buttonController.setupInterruptPin(i, CHANGE); // Setup each button as interrupt for INTA/B
    buttonController.pinMode(i, INPUT_PULLUP);
  }

  // Clear interrupts on the IC
  buttonController.clearInterrupts();

  // Initialise the MCU 'interrupt' pin
  pinMode(MCU_INT, INPUT_PULLUP);
}

// Function to read all buttons from MCP23017 chip and write as a 2-byte packet
void readWriteButtons(void)
{
    // Only read buttons if 'interrupt' pin is active low (ie. button state changed)
    if (!digitalRead(MCU_INT))
    {
        // Obtain button packet data from IC
        uint16_t buttonPacket = buttonController.readGPIOAB();
        uint16_t packedData = 0; // This will hold the bit-packed button states

        // Update button state from new packet
        for (int i = 0; i < NUM_BUTTONS; i++)
        {
            bool currentState = !getBit(buttonPacket, i); // Decode & update button state from bit-wise operation

            // Store button state into the packed data variable
            if (currentState) 
            {
                packedData |= (1 << i); // Set bit if button is pressed
            }

            // Check if button state has changed
            if (currentState != buttons[i].state)
            {
                buttons[i].state = currentState; // Update the button state

                // Debugging messages (Optional)
                if(enableDebug)
                {
                  Serial.print("@PlutoPAD: ");
                  Serial.print(buttons[i].name);
                  Serial.println(currentState ? " pushed" : " released");
                }
            }
        }

        // Send data only if a Bluetooth client is connected
        if (client_isConnected)
        {
            serialBluetooth.write((uint8_t*)&packedData, sizeof(packedData)); // Send the 2-byte packet via BLUETOOTH
        }

        // Clear interrupts on the MCP23017 chip
        buttonController.clearInterrupts();
    }
}

// Function to check if a Bluetooth Client is connected and sets the flag accordingly
void checkBluetoothClient(void)
{
  if (serialBluetooth.hasClient()) 
  {
    // A Bluetooth Client is connected
    if (!client_isConnected) 
    {
      client_isConnected = true;
      currentLEDMode = LED_SOLID;
      Serial.println("@PlutoPAD: Bluetooth Client connected!");
    }
  } else 
  {
    // No Bluetooth Client connected
    if (client_isConnected) 
    {
      currentLEDMode = LED_PULSE;
      client_isConnected = false;
      Serial.println("@PlutoPAD: Bluetooth Client disconnected!");
    }
  }
}

// Function to enable game controller wired mode
void enableWiredMode(void)
{
  // Enable game controller wired mode
  Serial.println("@PlutoPAD: [WIRED] mode activated!");
  Serial.println("@PlutoPAD:  * Bluetooth connection disabled! *");

  // Future implementation for game controller wired mode
  // here...
}

// Function to get the value of a specific bit in a 16-bit number (from button packet)
//  - Used to decode the button packet data from MCP23017 chip into states
uint16_t getBit(uint16_t num, int bitPosition) 
{
    // Ensure that the bitPosition is within valid range (0 to 15 for uint16_t)
    if (bitPosition < 0 || bitPosition > 15) 
    {
        return 0;   // return a default value
    }
 
    // Use bitwise AND to extract the desired bit
    // Shift 1 to the left by bitPosition and perform bitwise AND with num
    return (num & (1 << bitPosition)) >> bitPosition;
}

// Function to initialise the Status LED pin
void initStatusLED(void)
{
  pinMode(statusLED, OUTPUT);
  setStatusLED(0);

  // Startup Flash Sequence - Quick 2 flashes
  setStatusLED(255);
  delay(100);
  setStatusLED(0);
  delay(100);
  setStatusLED(255);
  delay(100);
  setStatusLED(0);
}

// Function to set the Status LED brightness
void setStatusLED(uint8_t brightness) 
{
  // Set the Status LED color by writing to the pin
  analogWrite(statusLED, brightness);
}

// Function to update the Status LED based on the current selected mode
void updateStatusLED(void)
{
  static unsigned long lastUpdate = 0;  // Last update time
  static int brightness = 0;            // LED brightness
  static int fadeDir = 1;               // Fade direction (1 for increasing, -1 for decreasing)
  static bool ledState = false;         // LED state (on/off)
  unsigned long now = millis();         // Current time

  // Determine which LED mode should be active, prioritising the battery critical override
  LEDMode activeMode = (batteryLEDOverride != LED_OFF) ? batteryLEDOverride : currentLEDMode;

  switch (activeMode)
  {
    case LED_OFF:
    {
      setStatusLED(0);
      break;
    }
    case LED_SOLID:
    {
      setStatusLED(100);
      break;
    }
    case LED_PULSE:
    {
      int pulseInterval = 10;  // Slower updates (was 5)
      int fadeStep = 5;        // Smaller changes (was 15)

      if (now - lastUpdate >= pulseInterval)
      {
        lastUpdate = now;
        brightness += fadeStep * fadeDir;

        if (brightness <= 0 || brightness >= 255)
        {
          fadeDir *= -1;
          brightness = constrain(brightness, 0, 255);
        }

        setStatusLED(brightness);
      }
      break;
    }
    case LED_SLOW_BLINK:
    {
      if (now - lastUpdate >= 500) // Distinct slow blinking
      {
        lastUpdate = now;
        ledState = !ledState;
        setStatusLED(ledState ? 255 : 0);
      }
      break;
    }
    case LED_FAST_BLINK:
    {
      if (now - lastUpdate >= 120) // Distinct fast blinking
      {
        lastUpdate = now;
        ledState = !ledState;
        setStatusLED(ledState ? 255 : 0);
      }
      break;
    }
    case LED_DOUBLE_BLINK:
    {
      handleBlinkPattern(2, 200, 1000); // 2 blinks, 200ms per toggle, 1s pause
      break;
    }
    case LED_TRIPLE_BLINK:
    {
      handleBlinkPattern(3, 200, 1000); // 3 blinks, 200ms per toggle, 1s pause
      break;
    }
  }
}

// Function to handle the blink pattern for the Status LED
void handleBlinkPattern(uint8_t numBlinks, uint16_t blinkInterval, uint16_t pauseDuration)
{
  static unsigned long lastUpdate = 0;
  static int blinkCount = 0;
  static bool inPause = false;
  static bool ledState = false;
  static unsigned long pauseStart = 0;

  unsigned long now = millis();

  if (!inPause && now - lastUpdate >= blinkInterval)
  {
    lastUpdate = now;
    ledState = !ledState;

    if (ledState)
    {
      setStatusLED(100); // LED on
    }
    else
    {
      setStatusLED(0);   // LED off
      blinkCount++;

      if (blinkCount >= numBlinks)
      {
        blinkCount = 0;
        inPause = true;
        pauseStart = now;
      }
    }
  }
  else if (inPause && now - pauseStart >= pauseDuration)
  {
    inPause = false;
  }
}

// Function to initialise the INA226 chip to read 1S LiPo battery voltage
void initBatteryMonitor(void)
{
  // Initialise the INA226 battery voltage monitor
  if (!INA.begin()) 
  {
    Serial.println("@PlutoPAD: *ERROR* Battery Monitor Uninitialised! ... Fix and Reboot!");
    while (1);
  }
  INA.setMaxCurrentShunt(1, 0.01); // Set maximum shunt current
}

// Function to return the 1S LiPo battery voltage from the INA226 chip
float readBatteryVoltage(void)
{
  // Read the battery voltage from the INA226 chip
  float batteryVoltage = INA.getBusVoltage();

  // Debugging messages (Optional)
  if(enableDebug)
  {
    Serial.print("@PlutoPAD: Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println("V");
  }

  return batteryVoltage;
}

// Function to check the battery voltage and update the LED mode accordingly
void checkBatteryVoltage(void)
{
  unsigned long currentMillis = millis();  // Current time

  // Check if interval have passed since the last battery check
  if(currentMillis - prevBatteryCheck >= batteryCheckInterval || startupBatteryCheck) 
  {
    prevBatteryCheck = currentMillis; // Update the last check time
    startupBatteryCheck = false;      // Reset the startup check flag

    float batteryVoltage = readBatteryVoltage();  // Battery voltage

    // Check battery voltage and update LED mode
    if (batteryVoltage < CRITICAL_VOLTAGE && isBatteryConnected(batteryVoltage))
    {
      Serial.println("@PlutoPAD: *** BATTERY CRITICAL -- CHARGE NOW! ***");
      lowPowerMode(); // Enter low power mode
    } 
    else if (batteryVoltage <= LOW_VOLTAGE && isBatteryConnected(batteryVoltage))
    {
      Serial.println("@PlutoPAD: Battery Low! Please charge soon...");
      batteryLEDOverride = LED_FAST_BLINK; // Override LED mode to FAST blink
    } 
    else
    {
      batteryLEDOverride = LED_OFF; // Reset the LED Override to OFF
    }
  }
}

// Function to check if battery is connected or not, assumes voltage near 0V is no battery conencted
bool isBatteryConnected(float voltage) 
{
  return voltage > 0.1; // Assumes battery can never be 0.1V, therefore no battery connected
}

// Function to enter low power mode if the battery voltage is critical
//  - This function will pulse the LED five time, then enter deep sleep mode
void lowPowerMode(void)
{
  // Set the LED mode to OFF and enter deep sleep
  currentLEDMode = LED_OFF;
  batteryLEDOverride = LED_OFF;
  Serial.println("@PlutoPAD: *** ENTERING LOW POWER MODE *** ");
  float batteryVoltage = readBatteryVoltage(); // Read the battery voltage

  if (batteryVoltage >= RECOVERY_VOLTAGE)
  {
    Serial.println("@PlutoPAD: Battery OK! Exiting low power mode...");
    return;
  }

  // Pulse LED five times
  setStatusLED(255);
  delay(100);
  setStatusLED(0);
  delay(100);
  setStatusLED(255);
  delay(100);
  setStatusLED(0);
  delay(100);
  setStatusLED(255);
  delay(100);
  setStatusLED(0);
  delay(100);
  setStatusLED(255);
  delay(100);
  setStatusLED(0);
  delay(100);
  setStatusLED(255);
  delay(100);
  setStatusLED(0);
  
  Serial.println("@PlutoPAD: *** VOLTAGE CRITICAL! SLEEPING... ***");

  // Enter deep sleep mode for WAKE_INTERVAL seconds
  esp_sleep_enable_timer_wakeup(WAKE_INTERVAL * 1000000ULL);
  esp_deep_sleep_start();
}