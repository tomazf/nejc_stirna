#include <Wire.h>
#include <PCF8575.h>
#include <Button2.h>
#include <EEPROM.h>
#include <Streaming.h>
#include <CircularBuffer.hpp>

// Ultrasonic sensor and LED configurations
#define PCF8575_ADDR 0x20     // I2C address of the PCF8575
#define BAUD_RATE 9600
#define BUFFER_SIZE 128
#define SENSOR_LOW 200
#define SENSOR_HIGH 4500

PCF8575 pcf(PCF8575_ADDR);

// Calibration and input
//#define TRIMMER_PIN A0      // not used
#define BTN_MIN_PIN D3
#define BTN_MAX_PIN D5
#define EEPROM_SIZE 2
#define BTN_DOUBLECLICK_MS 300
#define MIN_BLINKTIME 500
#define SERIAL_BUFFER_SIZE 64             // Typical size of the serial buffer

// Constants
const uint32_t MEASURE_INTERVAL = 30000;  // 30 seconds
const int LED_COUNT = 12;
const int EEPROM_MIN_ADDR = 0;            // EEPROM address for minLevel
const int EEPROM_MAX_ADDR = 1;            // EEPROM address for maxLevel

// Variables
uint8_t minLevel = 0, maxLevel = 255;     // Calibration levels
unsigned long lastMeasureTime = 0;
unsigned long lastSerialReadTime = 0;     // Time of the last successful read
unsigned long previousMillis = 0;         // Stores the last time the LED was toggled

// Calibration mode
bool inCalibrationMode = false;
bool calibratingMin = false;
bool receivedData = true;
bool minBlink = false;
bool pinState = LOW;
int distanceSum = 0;
int measurementCount = 0;
bool debugEnabled = true;                           // Set to true to enable debug messages
uint8_t currentMapped = 0;

// for timers
unsigned long lastAverageTime = 0;
const unsigned long averagingInterval = 10000;      // 10 seconds
unsigned long calibrationStartTime = 0;             // Tracks when calibration mode starts
const unsigned long calibrationTimeout = 30000;     // 30-second timeout in milliseconds

// Button objects
Button2 buttonMin(BTN_MIN_PIN);
Button2 buttonMax(BTN_MAX_PIN);

// buffer
CircularBuffer<uint8_t, BUFFER_SIZE> ringBuffer;    // Ring buffer for received data

void setup() {
  // Initialize serial communication for sensor
  Serial.begin(BAUD_RATE);
  delay(200);
  Serial.println();
  Serial.println("SETUP start...");

  EEPROM.begin(EEPROM_SIZE);

  // Initialize I2C for PCF8575
  Wire.begin();
  pcf.begin();

  // Set PCF8575 pins as outputs
  for (int i = 0; i < LED_COUNT; i++) {
    pcf.pinMode(i, OUTPUT);
    pcf.digitalWrite(i, HIGH);                      // Turn LEDs off (active low)
  }

  // Attach button callbacks
  buttonMin.setDoubleClickTime(BTN_DOUBLECLICK_MS);
  buttonMax.setDoubleClickTime(BTN_DOUBLECLICK_MS);
  buttonMin.setDoubleClickHandler(handleMinDoublePress);
  buttonMax.setDoubleClickHandler(handleMaxDoublePress);

  // Read calibration data from EEPROM
  minLevel = EEPROM.read(EEPROM_MIN_ADDR) | (EEPROM.read(EEPROM_MIN_ADDR + 1) << 8);
  maxLevel = EEPROM.read(EEPROM_MAX_ADDR) | (EEPROM.read(EEPROM_MAX_ADDR + 1) << 8);

  // display min and max from EEPROM
  Serial << "vaules from memory - MIN: " << minLevel << " MAX: " << maxLevel;
  Serial.println();
  Serial.println("SETUP done!");

  // DEMO
  runLEDDemo();
  Serial.println("DEMO done!");
}

void loop() {
  // Process button events
  buttonMin.loop();
  buttonMax.loop();

  // if no data is received
  if (!receivedData && !inCalibrationMode) knightRiderEffect(70);

  // Handle calibration mode
  if (inCalibrationMode) {
    handleCalibration();
  }

  // read Serial
  checkSerial();

  // Process received packets
  processPackets();

  //doAverage
  doAverage();

  //do MinBlink
  if (minBlink) doMinBlink();
}

void checkSerial() {
  // Read data asynchronously from hardware serial
  while (Serial.available() > 0) {
    uint8_t byte = Serial.read();
    ringBuffer.push(byte);                        // Add byte to the ring buffer
  }
}

void doAverage() {
  // Check if data is being received
  if (millis() - lastAverageTime >= averagingInterval) {
    if (measurementCount == 0 && debugEnabled) {
      receivedData = false;
      minBlink = false;
      Serial.println("Warning: No valid data received in the last 10 seconds.");
    }
    else
    {
      // Calculate and output average every 10 seconds
      uint8_t average = calculateMappedAverage();
      if (!inCalibrationMode) updateLEDs(average);        // Update the LED display based on the distance
      else
      {
        if (calibratingMin)
          Serial.print("calibration MAX: ");
        else
          Serial.print("calibration MIN: ");
        Serial.println(currentMapped);
      }
    }

    // Reset the averaging variables
    distanceSum = 0;
    measurementCount = 0;
    lastAverageTime = millis();
  }
}

void processPackets() {
  // Ensure at least 4 bytes are available to form a packet
  while (ringBuffer.size() >= 4) {
    // Peek at the first byte to check for packet start
    if (ringBuffer[0] == 0xFF) {
      // Extract packet
      uint8_t hiByte = ringBuffer[1];
      uint8_t loByte = ringBuffer[2];
      uint8_t crc = ringBuffer[3];

      // Valid packet, calculate distance
      uint16_t distance = (hiByte << 8) | loByte;

      // Accumulate for averaging
      distanceSum += distance;
      measurementCount++;

      // Remove 4 bytes of the processed packet
      for (int i = 0; i < 4; i++) {
        ringBuffer.shift();
      }
    } else {
      // Discard the first byte if it does not indicate the start of a packet
      if (debugEnabled) {
        Serial.println("Invalid start byte, discarding.");
      }
      ringBuffer.shift();
      receivedData = false;
    }
  }
}

uint8_t calculateMappedAverage() {
  // Return 0 if no measurements
  if (measurementCount == 0) {
    return 0;
  }

  receivedData = true;
  uint16_t averageDistance = distanceSum / measurementCount;          // Calculate average distance

  // Map distance to 0–255 range
  uint8_t mapped = map(averageDistance, SENSOR_LOW, SENSOR_HIGH, 0, 255);
  if (debugEnabled) {
    Serial << "avg (mapped): " << mapped << " (" << averageDistance / 10 << " cm)";
    Serial.println();
  }

  currentMapped = mapped;
  return mapped;
}

void updateLEDs(int distance) {
  int level = 0;

  // Ensure minLevel and maxLevel are valid
  if (minLevel < maxLevel && distance >= 0) {
    level = map(distance, minLevel, maxLevel, LED_COUNT, 0);      // do reverse LED_COUNT here if high/low
    // Constrain the result to the valid range
    level = constrain(level, 0, LED_COUNT);

    // for blink if too low
    if (distance > maxLevel)
    {
      minBlink = true;
      clearLED();
    }
    else minBlink = false;
  }
  else
  {
    // Handle invalid calibration data
    Serial.println("Invalid MIN and MAX level!");
    Serial.println("Invalid calibration data or distance. Check minLevel and maxLevel!");

    // first&last LED ON
    clearLED();
    pcf.digitalWrite(0, LOW);               // LED ON
    pcf.digitalWrite(LED_COUNT - 1, LOW);   // LED ON
    return;
  }

  if (debugEnabled) {
    Serial << "CURR: minLevel: " << minLevel << " maxLevel: " << maxLevel << " distance: " << distance;
    Serial.println();
  }

  // first LED is always ON
  if (!minBlink)
  {
    pcf.digitalWrite(0, LOW);
    // Update LEDs based on the level
    for (int i = 1; i < LED_COUNT; i++) {
      if (i < level) {
        pcf.digitalWrite(i, LOW); // LED ON
      } else {
        pcf.digitalWrite(i, HIGH); // LED OFF
      }
    }
  }
}

void doMinBlink() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= MIN_BLINKTIME) {
    previousMillis = currentMillis;

    pinState = !pinState;                           // Flip the state
    pcf.digitalWrite(0, pinState);
  }
}

void handleMinDoublePress(Button2 &btn) {
  if (!inCalibrationMode) {
    enterCalibrationMode(true);                     // Enter calibration mode for MIN
  } else if (calibratingMin) {
    exitCalibrationMode(true);                      // Exit calibration mode
  }
}

void handleMaxDoublePress(Button2 &btn) {
  if (!inCalibrationMode) {
    enterCalibrationMode(false);                    // Enter calibration mode for MAX
  } else if (!calibratingMin) {
    exitCalibrationMode(true);                      // Exit calibration mode
  }
}

void enterCalibrationMode(bool isMin) {
  inCalibrationMode = true;
  calibrationStartTime = millis();                // Record the start time for timeout
  calibratingMin = isMin;

  clearLED();                                     // turn off all LEDs

  // Blink LEDs to indicate calibration mode
  if (!isMin) {
    blinkCalibrationLEDs(0, 4, 800);              // Blink lower 6 LEDs
  } else {
    blinkCalibrationLEDs(8, 12, 800);             // Blink upper 6 LEDs
  }
}

void exitCalibrationMode(bool manual) {
  if (manual)                                     //  manual indicates how we exit calibration mode
  {
    // Save calibration data before exiting calibration mode
    if (calibratingMin) {
      minLevel = getLevel();
      blinkCalibrationLEDs(0, 4, 100);
    } else {
      maxLevel = getLevel();
      blinkCalibrationLEDs(8, 12, 100);
    }
    saveCalibrationData();

    Serial << "min: " << minLevel << " - max: " << maxLevel << "\n";
    Serial.println("CALIBRATION data saved");
  }
  else
  {
    // timeout passed, exit calibration mode
    Serial.println("CALIBRATION mode EXIT - no data saved!");
  }

  // Now exit calibration mode
  inCalibrationMode = false;
}

uint8_t getLevel() {
  return currentMapped;                           // this gets calculated in doAverage()
}

void handleCalibration() {
  // Automatically exit calibration mode if timeout passed
  if (millis() - calibrationStartTime >= calibrationTimeout) {
    exitCalibrationMode(false);
  }

  // The actual saving happens in exitCalibrationMode()
}

void blinkCalibrationLEDs(int start, int end, int duration_time) {
  for (int i = 0; i < 3; i++) {                 // Blink three times
    for (int j = start; j < end; j++) {
      pcf.digitalWrite(j, LOW);                 // LED ON
    }
    delay(duration_time);
    for (int j = start; j < end; j++) {
      pcf.digitalWrite(j, HIGH);                // LED OFF
    }
    delay(duration_time);
  }
}

void saveCalibrationData() {
  // Save minLevel and maxLevel to EEPROM (1 byte each)
  EEPROM.write(EEPROM_MIN_ADDR, minLevel);                       // Store lower byte of minLevel
  EEPROM.write(EEPROM_MAX_ADDR, maxLevel);                       // Store lower byte of maxLevel
  EEPROM.commit();                                               // Commit the data to ensure it is written to flash
} 

void runLEDDemo() {
  // Slow Blink: All LEDs blink slowly
  for (int i = 0; i < 3; i++) {
    for (int led = 0; led < LED_COUNT; led++) {
      pcf.digitalWrite(led, LOW); // Turn LED ON
    }
    delay(500);
    for (int led = 0; led < LED_COUNT; led++) {
      pcf.digitalWrite(led, HIGH); // Turn LED OFF
    }
    delay(500);
  }

  // Fast Blink: All LEDs blink quickly
  for (int i = 0; i < 5; i++) {
    for (int led = 0; led < LED_COUNT; led++) {
      pcf.digitalWrite(led, LOW); // Turn LED ON
    }
    delay(100);
    for (int led = 0; led < LED_COUNT; led++) {
      pcf.digitalWrite(led, HIGH); // Turn LED OFF
    }
    delay(100);
  }

  // Turn All LEDs ON, then OFF
  for (int led = 0; led < LED_COUNT; led++) {
    pcf.digitalWrite(led, LOW); // Turn LED ON
  }
  delay(1000);
  for (int led = 0; led < LED_COUNT; led++) {
    pcf.digitalWrite(led, HIGH); // Turn LED OFF
  }
  delay(1000);

  // Blink from Both Ends to Center
  for (int i = 0; i < 3; i++) { // Repeat effect 3 times
    for (int j = 0; j < (LED_COUNT + 1) / 2; j++) {
      pcf.digitalWrite(j, LOW); // Turn outer LEDs ON
      pcf.digitalWrite(LED_COUNT - j - 1, LOW);
      delay(120 - i * 20);
      pcf.digitalWrite(j, HIGH); // Turn outer LEDs OFF
      pcf.digitalWrite(LED_COUNT - j - 1, HIGH);
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int led = 0; led < LED_COUNT; led++) {
      pcf.digitalWrite(led, LOW);                         // Turn LED ON
    }
    delay(50);
    for (int led = 0; led < LED_COUNT; led++) {
      pcf.digitalWrite(led, HIGH);                        // Turn LED OFF
    }
    delay(50);
  }

  // Ensure all LEDs are OFF at the end
  for (int led = 0; led < LED_COUNT; led++) {
    pcf.digitalWrite(led, HIGH);                          // Turn LED OFF
  }
}

void clearLED() {
  for (int i = 0; i < LED_COUNT; i++) {
    pcf.digitalWrite(i, HIGH);                            // Turn LED OFF
  }
}

void knightRiderEffect(unsigned long interval) {
  static int currentLED = 0;                              // Tracks the current LED index
  static int direction = 1;                               // 1 = forward, -1 = backward
  static unsigned long lastUpdate = 0;                    // Tracks the last update time

  // Check if the interval has elapsed
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= interval) {
    lastUpdate = currentTime;

    // Turn off all LEDs
    for (int i = 0; i < LED_COUNT; i++) {
      pcf.digitalWrite(i, HIGH);                          // Turn LED OFF (active LOW)
    }

    // Turn on the current LED
    pcf.digitalWrite(currentLED, LOW);                    // Turn LED ON (active LOW)

    // Update the LED index for the next step
    currentLED += direction;

    // Change direction if we reach the ends
    if (currentLED >= LED_COUNT) {
      currentLED = LED_COUNT - 2;             // Step back into range
      direction = -1;                         // Reverse direction
    } else if (currentLED < 0) {
      currentLED = 1;                         // Step back into range
      direction = 1;                          // Reverse direction
    }
  }
}
