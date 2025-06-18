#include <Wire.h> // Still useful if you have other I2C sensors, though not directly for built-in OLED
#include <TFT_eSPI.h> // Graphics and font library for Wio Terminal display
#include <SPI.h>    // Required for TFT_eSPI
#include <Adafruit_GFX.h> // Core graphics library (used by TFT_eSPI)
#include <Adafruit_NeoPixel.h> // For your external NeoPixel stick

// Initialize Wio Terminal's built-in display
TFT_eSPI tft = TFT_eSPI();

// Define NeoPixel parameters (RGB LED Stick)
// External NeoPixel stick is connected to Wio Terminal's D0 pin
#define LED_PIN 0        // WioTerminal Digital Pin D0 for RGB LED Stick
#define NUM_LEDS 20      // Number of LEDs on your stick
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Define sensor and control pins
// Grove Ultrasonic Distance Sensor is connected to Wio Terminal's D3 pin
#define ULTRASONIC_PIN 3 // Digital pin for Grove Ultrasonic Distance Sensor

// Define Wio Terminal buttons correctly
#define BUTTON_A WIO_KEY_C
#define BUTTON_B WIO_KEY_B
#define BUTTON_C WIO_KEY_A

// Wio Terminal Built-in Speaker - using proper speaker library
// According to Wio Terminal specs: ≥78dB @10cm 4000Hz

// Define detection thresholds and timings
const int DETECTION_DISTANCE_CM = 30; // Max distance to detect presence (cm)
// For a toilet, you might place the sensor in the doorway. Adjust this value
// based on testing to reliably detect a person passing through.
const unsigned long ENTRY_COOLDOWN_MS = 2000; // Cooldown after an entry detection (to avoid double-counting)
const unsigned long EXIT_INFERENCE_TIMEOUT_MS = 5000; // Time (ms) after no detection to infer exit
// This timeout assumes a person will be out of sensor range for at least this long after leaving.

const int DEBOUNCE_DELAY = 200; // Milliseconds for button debounce

bool isToiletOccupied = false; // True if toilet is occupied, False if available
unsigned long lastOccupancyActivityTime = 0; // Tracks when sensor last saw someone or when status changed
unsigned long lastEntryTime = 0; // Tracks when someone last entered (for cooldown)
unsigned long lastButtonPress = 0; // Track when button was last pressed

// Function Prototypes (declaration of functions defined later)
void displayMessage(String line1, String line2);
void setLEDStatus(byte r, byte g, byte b);
long readUltrasonicDistance(int triggerPin);
void updateToiletStatusDisplay(bool occupied); // Renamed for clarity
void playBeep();
void testSpeaker();

void setup() {
  Serial.begin(115200);

  // Initialize Wio Terminal's built-in display
  tft.begin();
  tft.setRotation(1); // Set display to landscape mode (0 for portrait)
  tft.fillScreen(TFT_BLACK); // Clear the screen
  tft.setTextColor(TFT_WHITE); // Set text color to white
  tft.setTextSize(2); // Set text size

  // Initialize NeoPixel
  pixels.begin();
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();  // Update the strip to turn all pixels off

  // Initialize Wio Terminal buttons correctly
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // Pin modes
  // Ultrasonic sensor pin will be configured for output/input dynamically within its function
  pinMode(ULTRASONIC_PIN, INPUT); 
  
  displayMessage("Toilet Status", "Initializing...");
  setLEDStatus(0, 0, 255); // Blue for startup
  
  // Test speaker during initialization
  Serial.println("Testing built-in speaker...");
  testSpeaker();
  
  delay(2000);
  updateToiletStatusDisplay(isToiletOccupied); // Show initial status (should be AVAILABLE)
  
  Serial.println("System initialized. Buttons: A=" + String(BUTTON_A) + " B=" + String(BUTTON_B) + " C=" + String(BUTTON_C));
  Serial.println("Using built-in speaker (no external connection needed)");
}

void loop() {
  // Read ultrasonic distance
  long distanceCm = readUltrasonicDistance(ULTRASONIC_PIN);

  // --- ENTRY DETECTION LOGIC ---
  // If a person is detected close to the sensor
  if (distanceCm > 0 && distanceCm < DETECTION_DISTANCE_CM) {
    // Always update last activity time when presence is detected.
    // This is crucial for the exit inference.
    lastOccupancyActivityTime = millis();

    if (!isToiletOccupied) { // If the toilet is currently AVAILABLE
      // Use a cooldown to prevent immediate re-trigger if the person just moves slightly
      // Check if enough time has passed since the last entry attempt
      if (millis() - lastEntryTime > ENTRY_COOLDOWN_MS) {
          isToiletOccupied = true; // Mark as occupied
          lastEntryTime = millis(); // Reset entry cooldown after a confirmed entry
          displayMessage("Toilet Status:", "OCCUPIED");
          setLEDStatus(255, 255, 0); // Yellow for occupied
          playBeep(); // Play entry beep
          delay(200); // Short beep
          Serial.println("Toilet now occupied (person entered).");
      }
    }
  }

  // --- EXIT INFERENCE LOGIC ---
  // If the toilet is currently OCCUPIED AND
  // no presence has been detected by the sensor for a set period (EXIT_INFERENCE_TIMEOUT_MS)
  // This check should *only* use lastOccupancyActivityTime, which is updated whenever *any* close object is seen.
  if (isToiletOccupied && (millis() - lastOccupancyActivityTime > EXIT_INFERENCE_TIMEOUT_MS)) {
    isToiletOccupied = false; // Mark as available
    displayMessage("Toilet Status:", "AVAILABLE");
    setLEDStatus(0, 255, 0); // Green for available
    
    // Hand Hygiene Reminder: This is the perfect spot for it!
    displayMessage("Cleanse Hands!", "Please use sanitizer!");
    setLEDStatus(0, 255, 255); // Cyan for sanitizer reminder
    playBeep(); 
    delay(3000); // Duration for sanitize reminder

    updateToiletStatusDisplay(isToiletOccupied); // Update display/LEDs to final available status
    Serial.println("Toilet now available (inferred exit & sanitize reminder).");
  }

  // --- Manual Reset Button ---
  // Read all button states
  bool btnAPressed = !digitalRead(BUTTON_A);
  bool btnBPressed = !digitalRead(BUTTON_B);
  bool btnCPressed = !digitalRead(BUTTON_C);

  // Debug button states (uncomment to troubleshoot)
  // Serial.println("Buttons: A=" + String(btnAPressed) + " B=" + String(btnBPressed) + " C=" + String(btnCPressed));

  // Handle button presses with proper debouncing
  if (btnAPressed || btnBPressed || btnCPressed) {
    if (millis() - lastButtonPress > DEBOUNCE_DELAY) {
      lastButtonPress = millis();
      
      // Perform reset action (any button can reset)
      isToiletOccupied = false; // Reset status to available
      lastEntryTime = millis(); // Reset cooldowns to prevent immediate re-trigger
      lastOccupancyActivityTime = millis(); 
      
      displayMessage("Toilet Reset!", "AVAILABLE");
      setLEDStatus(255, 100, 0); // Orange for reset
      playBeep(); // Play beep sound
      delay(500);
      updateToiletStatusDisplay(isToiletOccupied); // Update display/LEDs after reset
      Serial.println("Toilet manually reset to AVAILABLE.");
    }
  }
  
  delay(100); // Small delay to avoid busy-waiting and allow sensor readings
}

// Function to test speaker functionality
void testSpeaker() {
  Serial.println("Playing test beep on built-in speaker...");
  
  // Try multiple methods to get the speaker working
  
  // Method 1: Try different frequencies that work well with Wio Terminal
  tone(8, 4000, 500); // 4kHz for 500ms (matches Wio Terminal specs)
  delay(600);
  
  tone(8, 2000, 500); // 2kHz for 500ms
  delay(600);
  
  tone(8, 1000, 500); // 1kHz for 500ms
  delay(600);
  
  // Method 2: Try alternative pin if Method 1 doesn't work
  // tone(12, 4000, 500);
  // delay(600);
  
  // Method 3: Try DAC output for analog sound
  // analogWrite(DAC0, 128);
  // delay(500);
  // analogWrite(DAC0, 0);
  // delay(100);
  
  Serial.println("Speaker test complete");
}

// Function to display messages on Wio Terminal's built-in display
void displayMessage(String line1, String line2) {
  tft.fillScreen(TFT_BLACK); // Clear the screen
  tft.setCursor(0, 0); // Start text at top-left
  tft.println(line1);
  tft.setCursor(0, 30); // Move to next line (adjusted for text size 2)
  tft.println(line2);
  // No tft.display() needed as TFT_eSPI draws directly
}

// Function to update LED stick color
void setLEDStatus(byte r, byte g, byte b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

// Function to read distance from Grove Ultrasonic sensor
// Returns distance in centimeters, or 0 if no valid reading.
// NOTE: This assumes a single-pin ultrasonic sensor. If you have separate trigger/echo pins,
// you'll need to modify this function accordingly.
long readUltrasonicDistance(int triggerPin) {
  // Clear the trigger pin by setting it LOW for a short period
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Set the trigger pin HIGH for 10 microseconds to send a pulse
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  // Read the echo pin, returns the sound wave travel time in microseconds
  pinMode(triggerPin, INPUT); // Re-configure pin as input for echo
  long duration = pulseIn(triggerPin, HIGH, 25000); // Max 25ms timeout (approx 4m)
  
  // Calculate the distance: Speed of sound is 343 meters/second or 0.0343 cm/microsecond.
  // Divide by 2 because the sound travels to the object and back.
  long distanceCm = duration * 0.0343 / 2;
  
  // Filter out obviously bad readings (e.g., 0 or extremely high)
  if (distanceCm > 400 || distanceCm <= 0) { // Max range approx 400cm for typical ultrasonic
    return 0; // Invalid reading
  }
  return distanceCm;
}

// Function to update Wio Terminal display and LED based on toilet status
void updateToiletStatusDisplay(bool occupied) {
  tft.fillScreen(TFT_BLACK); // Clear the screen
  tft.setCursor(0, 0);
  tft.print("Toilet Status:");

  tft.setCursor(0, 30); // New line for status message (adjusted for text size 2)
  if (!occupied) { // If not occupied, it's AVAILABLE
    tft.println("AVAILABLE");
    setLEDStatus(0, 255, 0); // Green: Toilet is available
  } else { // If occupied
    tft.println("OCCUPIED");
    setLEDStatus(255, 255, 0); // Yellow: Toilet is occupied
  }
}

// Function to play a simple beep sound using the Wio Terminal's built-in speaker
void playBeep() {
  Serial.println("Playing beep on built-in speaker");
  
  // Use 4kHz frequency which matches Wio Terminal speaker specs (≥78dB @10cm 4000Hz)
  tone(8, 4000, 200); // Play 4kHz tone for 200ms
} 