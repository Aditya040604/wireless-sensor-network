#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define DHTPIN A2
#define DHTTYPE DHT11
#define SOILPIN A0
#define MQ5PIN  A1
#define FLAME_DIGITAL_PIN 52
#define BUZZER_PIN A8
#define GAS_THRESHOLD 40   // Fixed demo threshold
#define PUMP_IN1  30
#define PUMP_IN2  31

#define FAN_IN1   32
#define FAN_IN2   33

#define SOIL_DRY_THRESHOLD 30   // %
#define TEMP_THRESHOLD     30   // °C




// #define FLAME_ANALOG_PIN  A2



DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

float temperature = 0;
float humidity = 0;
int soilMoisture = 0;
int gasLevel = 0;

int flameDetected = 0;     // 0 = no fire, 1 = fire
// int flameIntensity = 0;   // 0–100 %


unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;

void setup() {
  Serial.begin(9600);        // Debug
  Serial1.begin(9600);       // HC-05

  dht.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(SOILPIN, INPUT);
  pinMode(MQ5PIN, INPUT);
  pinMode(FLAME_DIGITAL_PIN, INPUT);
  // pinMode(FLAME_ANALOG_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);
  pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);   // buzzer OFF


}


void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();

    readDHT();
    readSoil();
    readGas();
    readFlame();
    updateBuzzer();
    updateLCD();
    sendBluetoothData();
    controlPump();
    controlFan();

  }
}

// Temperature and humidity reading function
void readDHT() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT read failed");
  }
}

// Soil moisture readings
void readSoil() {
  int raw = analogRead(SOILPIN);
  soilMoisture = map(raw, 1023, 0, 0, 100);
  Serial.print(soilMoisture);
}

// Gas levels reading
void readGas() {
  int raw = analogRead(MQ5PIN);
  gasLevel = map(raw, 0, 1023, 0, 100);
}

// Lcd display function
void updateLCD() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C H:");
  lcd.print(humidity, 0);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Soil:");
  lcd.print(soilMoisture);
  lcd.print("% Gas:");
  lcd.print(gasLevel);
  lcd.print("% F:");
  lcd.print(flameDetected ? "Y" : "N");
}

// Bluetooth communication code
void sendBluetoothData() {
  String json = "{";
  json += "\"temperature\":" + String(temperature) + ",";
  json += "\"humidity\":" + String(humidity) + ",";
  json += "\"soil\":" + String(soilMoisture) + ",";
  json += "\"gas\":" + String(gasLevel);
  json += "\"flame\":" + String(flameDetected) + ",";
  // json += "\"flame_intensity\":" + String(flameIntensity);

  json += "}";

  Serial1.println(json);
}
// Flames detecting code
void readFlame() {
  // Digital flame detection (MOST IMPORTANT)
  // Most modules: LOW = flame detected
  flameDetected = digitalRead(FLAME_DIGITAL_PIN);

  // Analog flame intensity (optional)
  // int raw = analogRead(FLAME_ANALOG_PIN);
  // flameIntensity = map(raw, 1023, 0, 0, 100);

  // Debug
  if (flameDetected == HIGH) {   // <-- CHANGED HERE
      Serial.println("  -> Flame DETECTED");

    } else {
      Serial.println("  -> No Flame");
    }
  // Serial.print(" | Intensity: ");
  // Serial.print(flameIntensity);
  // Serial.println("%");
}

// Buzzer functioning code
void updateBuzzer() {
  if (flameDetected == 1 || gasLevel >= GAS_THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// Pump controlling function 
void controlPump() {
  if (soilMoisture < SOIL_DRY_THRESHOLD) {
    digitalWrite(PUMP_IN1, HIGH);
    digitalWrite(PUMP_IN2, LOW);   // Pump ON
  } else {
    digitalWrite(PUMP_IN1, LOW);
    digitalWrite(PUMP_IN2, LOW);   // Pump OFF (coast)
  }
}
void controlFan() {
  if (temperature > TEMP_THRESHOLD) {
    digitalWrite(FAN_IN1, HIGH);
    digitalWrite(FAN_IN2, LOW);    // Fan ON
  } else {
    digitalWrite(FAN_IN1, LOW);
    digitalWrite(FAN_IN2, LOW);    // Fan OFF
  }
}

