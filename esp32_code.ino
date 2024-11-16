#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Constants
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin (or -1 if sharing Arduino reset pin)
#define MS5611_ADDR 0x77 // MS5611 sensor address
#define CMD_RESET 0x1E
#define CMD_PROM_READ 0xA2
#define CMD_CONVERT_D1 0x44
#define CMD_ADC_READ 0x00
#define P0 20e-6 // Reference pressure for SPL calculations

// OLED Display Object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Global Variables
uint16_t C[6];   // Calibration coefficients
uint32_t D1 = 0; // Raw pressure data
double currentSPL = 0, peakSPL = 0;
unsigned long lastDisplayUpdate = 0; // For refresh rate control

// Median filter setup for SPL
#define FILTER_SIZE 7 // Number of SPL readings to keep for median filter
double splReadings[FILTER_SIZE];  // Array to store the last SPL readings
int splIndex = 0;  // Index to keep track of the current SPL position

// Function Prototypes
void resetSensor();
void readCalibrationData();
void startConversion(uint8_t cmd);
uint32_t readRawData();
void calculateSPL();
void updateDisplay();
void handleSerialInput();
double applyMedianFilter(double rawSPL);

void setup() {
  // Serial Monitor Initialization
  Serial.begin(115200);
  Wire.begin();

  // OLED Initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Halt execution
  }
  display.clearDisplay();
  display.display();

  // Sensor Initialization
  resetSensor();
  readCalibrationData();

  // Display Welcome Message
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();
  delay(2000);
}

void loop() {
  calculateSPL();
  if (millis() - lastDisplayUpdate >= 100) { // Update screen every 100 ms
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  handleSerialInput();
}

void resetSensor() {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(CMD_RESET);
  Wire.endTransmission();
  delay(3); // Allow reset to complete
}

void readCalibrationData() {
  for (uint8_t i = 0; i < 6; i++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(CMD_PROM_READ + (i * 2));
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, 2);

    if (Wire.available() == 2) {
      C[i] = (Wire.read() << 8) | Wire.read();
    }
  }
}

void startConversion(uint8_t cmd) {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
}

uint32_t readRawData() {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, 3);

  uint32_t result = 0;
  if (Wire.available() == 3) {
    result = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
  }
  return result;
}

void calculateSPL() {
  static double minPressure = 1e9, maxPressure = -1e9;
  const double pressureThreshold = 1e-3; // Minimum pressure difference for valid SPL calculation

  // Start pressure conversion
  startConversion(CMD_CONVERT_D1);
  delayMicroseconds(4540); // Conversion time for OSR=2048
  D1 = readRawData();

  // Calculate pressure
  int64_t OFF = ((int64_t)C[1] << 16);
  int64_t SENS = ((int64_t)C[0] << 15);
  double pressure = (((D1 * SENS) >> 21) - OFF) / 32768.0;

  // Update min and max pressure
  minPressure = min(minPressure, pressure);
  maxPressure = max(maxPressure, pressure);

  // Calculate pressure change
  double pressureChange = maxPressure - minPressure;

  // Check if pressureChange is above the threshold
  double rawSPL = 0;
  if (pressureChange > pressureThreshold) {
    rawSPL = 20 * log10(pressureChange / P0); // Valid SPL calculation
  } else {
    // If pressureChange is too small, retain the last valid SPL value
    rawSPL = currentSPL; // Or implement a more sophisticated fallback
  }

  // Apply median filter to the raw SPL calculation
  currentSPL = applyMedianFilter(rawSPL);

  // Update peak SPL
  peakSPL = max(peakSPL, currentSPL);

  // Reset min/max pressure periodically
  static unsigned long lastResetTime = 0;
  if (millis() - lastResetTime > 100) {
    minPressure = 1e9;
    maxPressure = -1e9;
    lastResetTime = millis();
  }
}

// Median filter function for SPL
double applyMedianFilter(double rawSPL) {
  // Add the new SPL reading to the array
  splReadings[splIndex] = rawSPL;
  splIndex = (splIndex + 1) % FILTER_SIZE;

  // Copy the SPL readings to a new array to sort them
  double sortedSPLs[FILTER_SIZE];
  memcpy(sortedSPLs, splReadings, sizeof(splReadings));

  // Sort the array
  for (int i = 0; i < FILTER_SIZE - 1; i++) {
    for (int j = i + 1; j < FILTER_SIZE; j++) {
      if (sortedSPLs[i] > sortedSPLs[j]) {
        double temp = sortedSPLs[i];
        sortedSPLs[i] = sortedSPLs[j];
        sortedSPLs[j] = temp;
      }
    }
  }

  // Return the median value (middle value in the sorted array)
  return sortedSPLs[FILTER_SIZE / 2];
}

void updateDisplay() {
  display.clearDisplay();

  // Display current SPL
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Current SPL:"));
  display.setTextSize(2);
  display.setCursor(0, 12); // Adjusted vertical position
  display.printf("%6.2f dB", currentSPL); // Fixed width with 2 decimal places

  // Display peak SPL
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.println(F("Peak SPL:"));
  display.setTextSize(2);
  display.setCursor(0, 48); // Adjusted vertical position
  display.printf("%6.2f dB", peakSPL); // Fixed width with 2 decimal places

  display.display();
}

void handleSerialInput() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'r') {
      peakSPL = 0; // Reset peak SPL
      Serial.println(F("Peak SPL reset!"));
    }
  }
}
