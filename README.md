# Seeed-Studio-Final-Project-Intern-

# Smart Restroom Occupancy & Hand Hygiene System

## 1. Overview and Objectives

This project introduces a Smart Restroom Occupancy & Hand Hygiene System, a contactless solution designed to enhance user experience and promote public health in modern shared environments. Built around the Seeed Wio Terminal, this system provides real-time restroom occupancy status and integrates an automated hand hygiene reminder. It utilises an ultrasonic sensor for discreet presence detection, an RGB LED stick for visual cues, and the Wio Terminal's built-in display and speaker for intuitive alerts, offering a practical and easy-to-deploy solution for various facilities.

### 1.2 Key Features

* **Contactless Occupancy Detection**: An ultrasonic sensor accurately determines if a restroom is occupied or available without physical interaction.
* **Real-time Visual Status**: A multi-colour RGB LED stick provides immediate, highly visible indication of the restroom's status (Available, Occupied, Reset, Reminder).
* **Intuitive Display Interface**: The Wio Terminal's integrated LCD screen displays clear text messages for detailed status updates.
* **Audio Alerts**: A built-in buzzer provides auditory feedback for status changes and reminders.
* **Automated Hand Hygiene Reminder**: A unique feature that prompts users to sanitise their hands upon exiting the restroom.
* **Manual Reset Functionality**: A physical button allows for manual resetting of the occupancy status for maintenance or exceptional circumstances.

### 1.3 Target Audience

This documentation is tailored for:

* IoT Enthusiasts and Makers
* Arduino and Wio Terminal Beginners
* Facility Managers
* Students and Educators

### 1.4 Learning Outcomes

By following this guide, you will learn to:

* Interface with the Seeed Wio Terminal and utilise its integrated display, speaker, and buttons.
* Integrate and read data from a Grove Ultrasonic Distance Sensor.
* Control and program a Grove RGB LED Stick for dynamic visual feedback.
* Implement state management logic for occupancy detection (entry/exit inference).
* Incorporate practical features like debounced button inputs and timed alerts.
* Develop a project with clear practical value and a positive impact on user behaviour.

## 2. Materials and Setup

To replicate this project, you will need the following hardware components and software tools.

### 2.1 Hardware List

| Component                             | Quantity | Role in Project                                                              | Connection Type (Grove)                                   |
| :------------------------------------ | :------- | :--------------------------------------------------------------------------- | :-------------------------------------------------------- |
| Wio Terminal                          | 1        | The central microcontroller processes sensor data and controls outputs.      | Host for internal components and external Grove modules via pins/ports |
| Grove Ultrasonic Distance Sensor      | 1        | To detect the presence of people approaching or entering the space.          | Digital (connected to Wio Terminal's WIO\_D3 pin)  |
| Grove RGB LED Stick (20-WS2813 Mini)  | 1        | For highly visible, colour-coded occupancy status and hygiene reminders.      | Digital (connected to Wio Terminal's WIO\_D0 pin)  |
| Wio Terminal Built-in Speaker         | 1        | Provides audible alerts for sanitiser reminders and occupancy resets.        | Internal (accessed via Speaker library)       |
| Wio Terminal Built-in Button (WIO\_KEY\_A) | 1        | Allows manual reset of the occupancy count.                                  | Internal (accessed via digitalRead(WIO\_KEY\_A)) |
| Wio Terminal Built-in TFT Display     | 1        | Displays real-time occupancy count and status messages.                      | Internal (accessed via TFT\_eSPI library)     |
| Grove Wires                           | 2        | For all component connections to the Wio Terminal.                           | Facilitate Digital connections (for Ultrasonic and NeoPixel) |
| Optional: Enclosure                   | 1        | A custom 3D-printed or off-the-shelf enclosure can provide a neat and protective casing for deployment. | -                                                         |

### 2.2 Software Setup

Before uploading the code to your Wio Terminal, you'll need to set up your Arduino IDE and install the necessary board support package and libraries.

1.  **Arduino IDE Installation**:
    * Download and install the Arduino IDE from the official Arduino website: [https://www.arduino.cc/en/software/](https://www.arduino.cc/en/software/)

<p align="center">
  <img src="inal%20Project%20Figure%201.png" alt="Arduino IDE Screenshot" width="500"/>
</p>
<p align="center">
  <em>Figure 1: A Screenshot of the Arduino IDE download page</em>
</p>

2.  **Wio Terminal Board Support Package**:
    * Open the Arduino IDE.
    * Go to `File > Preferences` (or `Arduino > Preferences` on macOS).
    * In the "Additional Boards Manager URLs" field, add the following URL: `https://files.seeedstudio.com/arduino/package_seeed_index.json`
    * Click `OK`.
    * Go to `Tools > Board > Boards Manager....`
    * Search for "Seeed SAMD" and install the "Seeed SAMD Boards" package. This includes support for the Wio Terminal.
    * Once installed, go to `Tools > Board > Seeed SAMD Boards` and select `Wio Terminal`.

3.  **Required Libraries Installation**:
    * In the Arduino IDE, go to `Sketch > Include Library > Manage Libraries....`
    * Search for and install the following libraries:
        * `TFT_eSPI`: Graphics and font library for Wio Terminal's display.
        * `Adafruit GFX Library`: Core graphics library, a dependency for TFT\_eSPI.
        * `Adafruit NeoPixel`: For controlling your NeoPixel RGB LED Stick.
    * `SPI.h` and `Wire.h` are standard Arduino libraries and typically come pre-installed with the board support package.

## 3. Hardware Connections

Accurate wiring is crucial for the proper functioning of your system. Follow the diagram and instructions below carefully.

### 3.1 Step-by-Step Connection Guide

1.  **Connect the Grove Ultrasonic Distance Sensor**:
    * Locate the Grove connector on your Wio Terminal labeled `D3` (Digital Pin 3).
    * Using a Grove cable, connect one end to the ultrasonic sensor and the other end to the `D3` Grove port on the Wio Terminal.

2.  **Connect the Grove RGB LED Stick**:
    * Identify the Data Input pin on your Grove RGB LED Stick (often labeled "DI" or an arrow pointing in).
    * Connect this Data Input pin to the Wio Terminal's Digital Pin `D0` (this is a standard header pin on the Wio Terminal, usually near the Grove connectors).
    * Connect the `5V` power pin of the Grove RGB LED Stick to a `5V` output pin on the Wio Terminal.
    * Connect the `GND` (ground) pin of the Grove RGB LED Stick to a `GND` pin on the Wio Terminal.

3.  **Internal Components (No External Wiring Needed)**:
    * The Wio Terminal's built-in speaker uses Digital Pin 8.
    * The Wio Terminal's built-in Button A, used for manual reset, is connected to Digital Pin 2.
    * These components are integrated, so no external wiring is required for them.

4.  **Powering the Wio Terminal**:
    * Connect the USB-C cable to the Wio Terminal's USB-C port and the other end to your computer or a 5V USB power adapter.

## 4. Software Implementation: Code Walkthrough

The Arduino code manages the entire logic of the Smart Restroom Occupancy System. This section explains each part of the code, detailing its function and how it contributes to the overall system.

### 4.1 Uploading the Code

1.  **Open the Sketch**: Open the provided Arduino sketch file (.ino) in the Arduino IDE.
2.  **Select Board and Port**: Go to `Tools > Board > Seeed SAMD Boards` and select `Wio Terminal`. Then, go to `Tools > Port` and select the serial port corresponding to your connected Wio Terminal.
3.  **Upload**: Click the "Upload" button (right arrow icon) in the Arduino IDE to compile and upload the code to your Wio Terminal.

### 4.2 Code Explanation

The code is structured into global definitions, `setup()` for initialization, and `loop()` for continuous operation, supported by several helper functions.

#### 4.2.1 Libraries and Pin Definitions

```cpp
#include <Wire.h>          // Still useful if you have other I2C sensors, though not directly for built-in OLED
#include <TFT_eSPI.h>      // Graphics and font library for Wio Terminal display
#include <SPI.h>           // Required for TFT_eSPI
#include <Adafruit_GFX.h>  // Core graphics library (used by TFT_eSPI)
#include <Adafruit_NeoPixel.h> // For your external NeoPixel stick

// Initialize Wio Terminal's built-in display
TFT_eSPI tft = TFT_eSPI();

// Define NeoPixel parameters (RGB LED Stick)
#define LED_PIN 0          // WioTerminal Digital Pin D0 for RGB LED Stick
#define NUM_LEDS 20        // Number of LEDs on your stick
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Define sensor and control pins
#define ULTRASONIC_PIN 3   // Digital pin for Grove Ultrasonic Distance Sensor
#define BUZZER_PIN 8       // Define the pin for the built-in speaker/buzzer for tone() function
#define RESET_BUTTON 2     // Use Wio Terminal's built-in button A (often maps to pin 2)
````

  * **Libraries**: The code includes necessary libraries for the Wio Terminal's display (`TFT_eSPI`, `SPI`, `Adafruit_GFX`), and the Grove RGB LED Stick (`Adafruit_NeoPixel`). `Wire.h` is included as a common I2C library, though not directly used for the primary components here.
  * **TFT\_eSPI tft**: Initializes the object for controlling the Wio Terminal's built-in LCD.
  * **NeoPixel Definitions**: `LED_PIN` (D0) specifies where the NeoPixel data line is connected, and `NUM_LEDS` (20) defines the count of LEDs on your stick. `pixels` is the object that manages the NeoPixel strip.
  * **Pin Definitions**: `ULTRASONIC_PIN` (D3) for the Grove sensor, `BUZZER_PIN` (D8) for the internal speaker, and `RESET_BUTTON` (D2) for the Wio Terminal's built-in button A. Using `#define` makes pin assignment clear and easy to modify.

#### 4.2.2 System Constants and State Variables

```cpp
const int DETECTION_DISTANCE_CM = 30;       // Max distance to detect presence (cm)
const unsigned long ENTRY_COOLDOWN_MS = 2000; // Cooldown after an entry detection (to avoid double-counting)
const unsigned long EXIT_INFERENCE_TIMEOUT_MS = 5000; // Time (ms) after no detection to infer exit
const int DEBOUNCE_DELAY = 50;              // Milliseconds for button debounce

bool isToiletOccupied = false;              // True if toilet is occupied, False if available
unsigned long lastOccupancyActivityTime = 0; // Tracks when sensor last saw someone or when status changed
unsigned long lastEntryTime = 0;             // Tracks when someone last entered (for cooldown)
bool buttonPressed = false;                 // To track button state and rising edge
```

  * `DETECTION_DISTANCE_CM` (30 cm): Defines the maximum distance at which the ultrasonic sensor considers something "present." This value should be calibrated based on the sensor's placement.
  * `ENTRY_COOLDOWN_MS` (2000 ms): Prevents rapid re-triggering after an entry detection.
  * `EXIT_INFERENCE_TIMEOUT_MS` (5000 ms): If no presence is detected for this duration while the toilet is marked OCCUPIED, the system infers an "exit."
  * `DEBOUNCE_DELAY` (50 ms): Used for the manual reset button to prevent multiple presses from a single physical press.
  * **State Variables**:
      * `isToiletOccupied`: A boolean flag (true/false) representing the current occupancy status.
      * `lastOccupancyActivityTime`: Stores the `millis()` timestamp of the last moment the ultrasonic sensor detected any object within `DETECTION_DISTANCE_CM`. Crucial for `EXIT_INFERENCE_TIMEOUT_MS` logic.
      * `lastEntryTime`: Stores the `millis()` timestamp of the last confirmed entry. Used for `ENTRY_COOLDOWN_MS`.
      * `buttonPressed`: A flag to manage button debouncing and detect the rising edge of a button press.

#### 4.2.3 Setup Function (`setup()`)

```cpp
void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(1);       // Set display to landscape mode (0 for portrait)
  tft.fillScreen(TFT_BLACK); // Clear the screen
  tft.setTextColor(TFT_WHITE); // Set text color to white
  tft.setTextSize(2);       // Set text size

  pixels.begin();
  pixels.clear();           // Set all pixel colours to 'off'
  pixels.show();            // Update the strip to turn all pixels off

  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(ULTRASONIC_PIN, INPUT);

  displayMessage("Toilet Status", "\nInitializing...");
  setLEDStatus(0, 0, 255);  // Blue for startup
  delay(2000);
  updateToiletStatusDisplay(isToiletOccupied); // Show initial status (should be AVAILABLE)
}
```

  * **Serial Communication**: `Serial.begin(115200)` initializes serial communication for debugging output.
  * **Display Initialisation**: `tft.begin()` initialises the Wio Terminal's display. `tft.setRotation(1)` sets it to landscape mode. `fillScreen()`, `setTextColor()`, and `setTextSize()` configure the display's appearance.
  * **NeoPixel Initialisation**: `pixels.begin()` initialises the NeoPixel strip. `pixels.clear()` sets all LEDs to off, and `pixels.show()` updates the physical strip.
  * **Pin Modes**: `pinMode(RESET_BUTTON, INPUT_PULLUP)` configures the button pin with an internal pull-up resistor. `pinMode(ULTRASONIC_PIN, INPUT)` sets the ultrasonic pin.
  * **Initial Status**: The system displays an "Initializing..." message, shows blue LEDs, then updates to the initial AVAILABLE status.

#### 4.2.4 Main Loop (`loop()`)

```cpp
void loop() {
  long distanceCm = readUltrasonicDistance(ULTRASONIC_PIN);

  // ENTRY DETECTION LOGIC
  if (distanceCm > 0 && distanceCm < DETECTION_DISTANCE_CM) {
    lastOccupancyActivityTime = millis(); // Always update activity time

    if (!isToiletOccupied) { // If currently AVAILABLE
      if (millis() - lastEntryTime > ENTRY_COOLDOWN_MS) { // Check cooldown
        isToiletOccupied = true; // Mark as occupied
        lastEntryTime = millis(); // Reset entry cooldown
        displayMessage("Toilet Status:", "\nOCCUPIED");
        setLEDStatus(255, 255, 0); // Yellow for occupied
        playBeep();
        delay(200);
        noTone(BUZZER_PIN);
        Serial.println("Toilet now occupied (person entered).");
      }
    }
  }

  // EXIT INFERENCE LOGIC
  if (isToiletOccupied && (millis() - lastOccupancyActivityTime > EXIT_INFERENCE_TIMEOUT_MS)) {
    isToiletOccupied = false; // Mark as available
    displayMessage("Cleanse Hands!", "\nPlease use sanitizer!");
    setLEDStatus(0, 255, 255); // Cyan for sanitizer reminder
    playBeep();
    delay(3000);
    noTone(BUZZER_PIN);
    updateToiletStatusDisplay(isToiletOccupied); // Update display/LEDs to final available status
    Serial.println("Toilet now available (inferred exit & sanitize reminder).");
  }

  // Manual Reset Button
  if (digitalRead(RESET_BUTTON) == LOW) { // Button pressed
    if (!buttonPressed) { // First press (rising edge)
      delay(DEBOUNCE_DELAY); // Debounce
      if (digitalRead(RESET_BUTTON) == LOW) { // Confirm press
        isToiletOccupied = false; // Reset status
        lastEntryTime = millis(); // Reset cooldowns
        lastOccupancyActivityTime = millis();
        displayMessage("Toilet Reset!", "\nAVAILABLE");
        setLEDStatus(255, 100, 0); // Orange for reset
        playBeep();
        delay(500);
        noTone(BUZZER_PIN);
        updateToiletStatusDisplay(isToiletOccupied);
        buttonPressed = true; // Mark button as pressed.
        Serial.println("Toilet manually reset to AVAILABLE.");
      }
    }
  } else {
    buttonPressed = false; // Reset button state when released
  }

  delay(100); // Small delay to avoid busy-waiting
}
```

  * **Ultrasonic Reading**: `readUltrasonicDistance()` is called at the beginning of each loop to get the current distance.
  * **Entry Detection**:
      * If `distanceCm` is within `DETECTION_DISTANCE_CM`, `lastOccupancyActivityTime` is updated, resetting the exit timer as long as someone is near the sensor.
      * If the toilet is `!isToiletOccupied` (available) and `ENTRY_COOLDOWN_MS` has passed, the system transitions to OCCUPIED. Visuals (yellow LEDs, "OCCUPIED" message) and an audio beep confirm this.
  * **Exit Inference & Hand Hygiene Reminder**:
      * If `isToiletOccupied` and no `lastOccupancyActivityTime` has occurred for `EXIT_INFERENCE_TIMEOUT_MS`, the system infers an exit.
      * The `isToiletOccupied` flag is set to `false`.
      * The hand hygiene reminder is activated: "Cleanse Hands\!" message, cyan LEDs, and a prolonged beep.
      * Finally, the status updates to AVAILABLE (green LEDs).
  * **Manual Reset Button**:
      * Checks if `RESET_BUTTON` is LOW (pressed).
      * The `if (!buttonPressed)` and `delay(DEBOUNCE_DELAY)` logic ensures only a single button press is registered per physical press.
      * Upon a confirmed press, `isToiletOccupied` is forced to `false`, and all activity/entry timers are reset. Orange LEDs and a beep indicate the reset, followed by the AVAILABLE status. `buttonPressed = false` resets the button state when it's released.
  * `delay(100)`: A small delay in the loop to prevent busy-waiting, reduce power consumption, and ensure stable sensor readings.

#### 4.2.5 Helper Functions

```cpp
void displayMessage(String line1, String line2) {
  tft.fillScreen(TFT_BLACK); // Clear the screen
  tft.setCursor(0, 0);       // Start text at top-left
  tft.println(line1);
  tft.setCursor(0, 15);      // Move to next line (adjust Y-offset as needed based on font size)
  tft.println(line2);
}

void setLEDStatus(byte r, byte g, byte b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

long readUltrasonicDistance(int triggerPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(triggerPin, INPUT);
  long duration = pulseIn(triggerPin, HIGH, 25000);
  long distanceCm = duration * 0.0343 / 2;
  if (distanceCm > 400 || distanceCm <= 0) {
    return 0; // Invalid reading
  }
  return distanceCm;
}

void updateToiletStatusDisplay(bool occupied) {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.print("Toilet Status:");
  tft.setCursor(0, 20);
  if (!occupied) {
    tft.println("\nAVAILABLE");
    setLEDStatus(0, 255, 0); // Green
  } else {
    tft.println("\nOCCUPIED");
    setLEDStatus(255, 255, 0); // Yellow
  }
}

void playBeep() {
  tone(BUZZER_PIN, 1000, 200); // Play 1kHz tone for 200ms
}
```

  * `displayMessage(String line1, String line2)`: Simplifies updating the Wio Terminal's display by clearing the screen, setting the cursor, and printing two lines of text.
  * `setLEDStatus(byte r, byte g, byte b)`: Sets all LEDs on the NeoPixel stick to a specified RGB color.
  * `readUltrasonicDistance(int triggerPin)`: Crucial for the ultrasonic sensor. It temporarily configures the `triggerPin` as an `OUTPUT` to send a 10µs trigger pulse, then reconfigures it as an `INPUT` to measure the echo pulse using `pulseIn()`. The distance is calculated using the speed of sound and filters out invalid readings.
  * `updateToiletStatusDisplay(bool occupied)`: Provides a consistent way to update both the display message and the NeoPixel LEDs based on the `isToiletOccupied` status. It shows "AVAILABLE" with green LEDs or "OCCUPIED" with yellow LEDs.
  * `playBeep()`: Uses the `tone()` command to play a 1kHz beep for 200ms on the `BUZZER_PIN`.

## 5\. Application & Future Enhancements

### 5.1 How to Use the System

Once assembled and programmed, the Smart Restroom Occupancy System operates autonomously:

1.  **Initial State (AVAILABLE)**: Upon startup or after a reset, the LED stick will glow Green, and the Wio Terminal display will show "Toilet Status: AVAILABLE".
2.  **Occupancy Detection (OCCUPIED)**: When a person passes within \~30 cm of the ultrasonic sensor, the system detects entry. The NeoPixel stick will change to Yellow, the Wio Terminal display will show "Toilet Status: OCCUPIED", and a short beep will sound.
3.  **Inferred Exit & Hand Hygiene Reminder**: If the restroom is occupied and the ultrasonic sensor detects no presence for 5 seconds (configurable), the system infers the person has exited. The NeoPixel stick will change to bright Cyan, the Wio Terminal display will show "Cleanse Hands\!" and "Please use sanitiser\!", and a longer beep will sound for 3 seconds. After the reminder, the system reverts to the Green / AVAILABLE state.
4.  **Manual Reset**: Pressing Wio Terminal's built-in Button A (mapped to D2) will manually reset the system to the Green / AVAILABLE state. The NeoPixel stick will briefly flash Orange, and the display will show "Toilet Reset\! AVAILABLE" with a distinct beep.

### 5.2 Practical Applications

The core principles of this system can be adapted for various smart space management scenarios:

  * Meeting Room Status Indicator
  * Private Office Occupancy
  * Resource Availability (e.g., study carrels, sound booths)
  * Queue Management
  * Home Automation (e.g., smart lighting or HVAC adjustments)

### 5.3 Possible Enhancements

This project serves as a robust foundation. Here are several avenues for future enhancement:

  * **Wi-Fi Connectivity & Cloud Dashboard**: Send occupancy data to a cloud platform for remote monitoring, historical data logging, and analytics.
  * **Multi-Sensor Fusion**: Incorporate additional sensors like PIR motion sensors or millimetre-wave radar for enhanced detection reliability.
  * **Custom Enclosure**: Design and 3D-print a sleek, wall-mountable enclosure for a professional deployment.
  * **Advanced Audio Prompts**: Utilise the `Speaker.h` library to play custom sound files or pre-recorded voice messages for detailed alerts.
  * **Power Optimisation**: Implement deep sleep modes for battery-powered deployment to extend operational life.
  * **Bi-directional Communication**: Allow remote control (e.g., reset status from a web dashboard) or integrate with building management systems.
  * **User Interface Improvements**: Add more sophisticated graphics to the display or integrate a small keypad for configurable settings.

## 6\. Troubleshooting & FAQs

This section addresses common issues you might encounter while building and testing your Smart Restroom Occupancy System.

### 6.1 Common Issues and Solutions

  * **Issue: My Wio Terminal display is blank or shows gibberish.**
      * **Solution**: Ensure the `TFT_eSPI` library is correctly installed. Verify `tft.begin()` and `tft.setRotation(1)` are called in `setup()`. Check the USB-C connection for stable power.
  * **Issue: The NeoPixel stick isn't lighting up, or the colours are incorrect.**
      * **Solution**: Double-check wiring: NeoPixel data line to Wio Terminal's Digital Pin `D0`, and `5V` and `GND`are  correctly wired. Ensure `NUM_LEDS` constant matches the actual number of LEDs. Large strips might require an external 5V power supply.
  * **Issue: The ultrasonic sensor gives inconsistent, 0, or very high readings.**
      * **Solution**: Verify Grove cable connection to D3 Grove port. Ensure no immediate obstructions in front of the sensor. Make sure the sensor is pointing towards the detection area, not at an angle causing echoes. Adjust `pulseIn()` timeout if a longer range is needed.
  * **Issue: The manual reset button (Button A) doesn't respond or triggers multiple times.**
      * **Solution**: Confirm `pinMode(RESET_BUTTON, INPUT_PULLUP);` is set in `setup()`. Ensure `DEBOUNCE_DELAY` is active; if multiple triggers persist, slightly increase `DEBOUNCE_DELAY` (e.g., to 100ms).
  * **Issue: The system doesn't detect exits properly, or the "Cleanse Hands\!" reminder triggers too early/late.**
      * **Solution**: Adjust `EXIT_INFERENCE_TIMEOUT_MS`. Increase it if it triggers too early, decrease it if too late. Ensure the ultrasonic sensor is positioned such that a person truly exits its detection range.

### 6.2 Frequently Asked Questions (FAQs)

  * **Q: Can I use a different type of ultrasonic sensor?**
      * **A**: Yes, but you might need to modify the `readUltrasonicDistance()` function if your sensor uses separate Trigger and Echo pins (like the HC-SR04) instead of a single shared pin like the Grove sensor.
  * **Q: How can I change the display font or size?**
      * **A**: The `TFT_eSPI` library offers various fonts and text size options. You can use `tft.setTextSize()` for scaling default fonts or `tft.setFreeFont()` for custom fonts. Refer to the TFT\_eSPI documentation for details.
  * **Q: Can I change the LED colours for different statuses?**
      * **A**: Absolutely\! Modify the RGB values (0-255 for Red, Green, Blue) in the `setLEDStatus()` function calls within your `loop()` and `updateToiletStatusDisplay()` functions to customise the colour scheme.
  * **Q: What if the sensor always detects something, even when the restroom is empty?**
      * **A**: This could be due to a permanent obstruction within the `DETECTION_DISTANCE_CM`. Adjust the sensor's position or reduce `DETECTION_DISTANCE_CM` to a value just beyond the obstruction but still capable of detecting a person.

## 7\. References & Acknowledgements

This project was developed using the Seeed Wio Terminal and Grove ecosystem, alongside widely used Arduino libraries.

  * **Seeed Studio**: Special thanks to Seeed Studio for providing the Wio Terminal and Grove sensors, which form the core of this project.
      * [Wio Terminal](https://www.seeedstudio.com/Wio-Terminal-p-4509.html)
      * [Grove - Ultrasonic Distance Sensor](https://www.google.com/search?q=https://www.seeedstudio.com/Grove-Ultrasonic-Distance-Sensor-p-746.html)
  * **Arduino IDE**: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)
  * **TFT\_eSPI Library**: [https://github.com/Bodmer/TFT\_eSPI](https://github.com/Bodmer/TFT_eSPI)
  * **Adafruit GFX Library**: [https://github.com/adafruit/Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library)
  * **Adafruit NeoPixel Library**: [https://github.com/adafruit/Adafruit\_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
