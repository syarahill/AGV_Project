#include <Wire.h>

// =============================================================================
// INA219 SETTINGS
// =============================================================================
#define INA219_ADDRESS 0x40    // I2C address
#define CONFIG_REG 0x00        // Configuration register
#define BUS_REG 0x02           // Bus voltage register

// Configuration values
#define RESET_CONFIG 0x8000    // Reset command
#define BATTERY_CONFIG 0x2000 | 0x0000 | 0x0780 | 0x0078 | 0x0007  // 32V range, 12-bit ADC

// =============================================================================
// BATTERY PARAMETERS
// =============================================================================
const float BATTERY_NOMINAL_MIN = 15.6;  // V
const float BATTERY_NOMINAL_MAX = 18.6;  // V
const float BATTERY_CHARGING_THRESHOLD = 21.5;  // V
const float BATTERY_VOLTAGE_JUMP = 3.0;  // V

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================
void writeRegister(uint8_t reg, uint16_t value);
uint16_t readRegister(uint8_t reg);
float getBatteryVoltage();
int getBatteryPercentage(float voltage);
String getBatteryStatus(float voltage);
bool isCharging(float voltage);

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  delay(1000);
  
  // Initialize INA219
  writeRegister(CONFIG_REG, RESET_CONFIG);
  delay(50);
  writeRegister(CONFIG_REG, BATTERY_CONFIG);
  delay(100);
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  static unsigned long lastReading = 0;
  
  if (millis() - lastReading >= 1000) {
    lastReading = millis();
    
    float voltage = getBatteryVoltage();
    int percentage = getBatteryPercentage(voltage);
    String status = getBatteryStatus(voltage);
    bool charging = isCharging(voltage);
    
    // Send data in structured format
    Serial.print("BAT:");
    Serial.print(voltage, 2);
    Serial.print(":");
    Serial.print(percentage);
    Serial.print(":");
    Serial.println(status);
    
    // Optional: Add warnings for low battery
    if (voltage < 16.0 && voltage > 1.0) {
      Serial.println("WARNING: LOW BATTERY - RETURN TO CHARGE");
    }
    if (voltage < 15.0 && voltage > 1.0) {
      Serial.println("CRITICAL: STOP AND CHARGE IMMEDIATELY");
    }
  }
  delay(10);
}

// =============================================================================
// INA219 FUNCTIONS
// =============================================================================
void writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(reg);
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t readRegister(uint8_t reg) {
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(INA219_ADDRESS, 2);
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  }
  return 0;
}

float getBatteryVoltage() {
  uint16_t busRegister = readRegister(BUS_REG);
  uint16_t voltageData = busRegister >> 3;       // Remove lower 3 status bits
  return voltageData * 0.004;                    // Convert to volts (4mV per bit)
}

// =============================================================================
// BATTERY STATUS FUNCTIONS
// =============================================================================
int getBatteryPercentage(float voltage) {
  if (voltage >= 20.0) return 100;      // Charging/Full
  else if (voltage >= 19.0) return 90;  // Very High
  else if (voltage >= 18.0) return 70;  // Good
  else if (voltage >= 17.0) return 50;  // Medium
  else if (voltage >= 16.0) return 30;  // Low
  else if (voltage >= 15.0) return 15;  // Very Low
  else if (voltage >= 14.0) return 5;   // Critical
  else return 0;                        // Dead/No battery
}

String getBatteryStatus(float voltage) {
  if (voltage >= BATTERY_CHARGING_THRESHOLD) return "CHARGING";
  else if (voltage >= 19.0) return "FULL";
  else if (voltage >= 18.0) return "GOOD";
  else if (voltage >= 17.0) return "MEDIUM";
  else if (voltage >= 16.0) return "LOW";
  else if (voltage >= 15.0) return "VERY LOW";
  else if (voltage >= 14.0) return "CRITICAL";
  else if (voltage >= 1.0) return "DEAD";
  else return "NO BATTERY";
}

bool isCharging(float voltage) {
  return voltage > 20.0;  // Makita 18V shows >20V when charging
}