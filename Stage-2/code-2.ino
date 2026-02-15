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

bool autoMode = false;   // default = automatic
bool pumpManualState = false;
bool fanManualState = false;


float temperature = 0;
float humidity = 0;
int soilMoisture = 0;
int gasLevel = 0;
int heartRate = 0;
int spO2 = 0;
bool pulseRequest = false;
unsigned long pulseStartTime = 0;
const unsigned long PULSE_DELAY = 15000; // 15 seconds



int flameDetected = 0;     // 0 = no fire, 1 = fire
// int flameIntensity = 0;   // 0–100 %
void sendBluetoothDataMIT()
{
  // Handle NaN cases

  // Format:
  // HeartRate,Temperature,Moisture,SpO2,Flame,Gas

  Serial1.print(heartRate);
  Serial1.print(",");

  Serial1.print(spO2);
  Serial1.print(",");

  Serial1.print(temperature);
  Serial1.print(",");

   Serial1.print(humidity);
  Serial1.print(",");

  Serial1.print(soilMoisture);
  Serial1.print(",");

  Serial1.print(flameDetected);
  Serial1.print(",");

  Serial1.println(gasLevel);   // IMPORTANT: println → sends \n

  // Optional: print to Serial Monitor
  Serial.print("Sent: ");
  Serial.print(heartRate);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(soilMoisture);
  Serial.print(",");
  Serial.print(spO2);
  Serial.print(",");
  Serial.print(flameDetected);
  Serial.print(",");
  Serial.println(gasLevel);
}



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
  if (!pulseRequest && millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();

    readDHT();
    readSoil();
    readGas();
    readFlame();
    updateBuzzer();
    updateLCD();
  
   
    sendBluetoothDataMIT();
   

  }
   if (Serial1.available())
    {
      char command = Serial1.read();
      Serial.println(command);

    if (command == 'A') {
      autoMode = true;
    }
    else if (command == 'M') {
      autoMode = false;
    }
      else if (command == 'P')
      {
        pumpManualState = true;
        Serial.println("Pump on");
      }
      else if (command == 'p')
      {
        pumpManualState = false;
        Serial.println("Pump off");
      }
       else if (command == 'F')
      {
        fanManualState = true;
        Serial.println("Fan on");
      }
       else if (command == 'f')
      {
        fanManualState = false;
        Serial.println("Fan off");
      }
      else if (command == 'H') {
  pulseRequest = true;
  pulseStartTime = millis();
}

    }
     if (pulseRequest && (millis() - pulseStartTime >= PULSE_DELAY)) {

    // Generate realistic values
    static int baseRate = 78;
    heartRate = baseRate + random(-3, 4);
    spO2 = 97 + random(-1, 2);

    Serial.println("Pulse value generated");

    // Send updated data to app
    sendBluetoothDataMIT();

    pulseRequest = false;  // reset
  }

     controlPump();
    controlFan();
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

  if (autoMode) {
    // Automatic Mode
    if (soilMoisture < SOIL_DRY_THRESHOLD) {
      digitalWrite(PUMP_IN1, HIGH);
      digitalWrite(PUMP_IN2, LOW);
    } else {
      digitalWrite(PUMP_IN1, LOW);
      digitalWrite(PUMP_IN2, LOW);
    }
  }

  else {
    // Manual Mode
    if (pumpManualState) {
      digitalWrite(PUMP_IN1, HIGH);
      digitalWrite(PUMP_IN2, LOW);
    } else {
      digitalWrite(PUMP_IN1, LOW);
      digitalWrite(PUMP_IN2, LOW);
    }
  }
}

void controlFan() {

  if (autoMode) {
    if (temperature > TEMP_THRESHOLD) {
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
    } else {
      digitalWrite(FAN_IN1, LOW);
      digitalWrite(FAN_IN2, LOW);
    }
  }

  else {
    if (fanManualState) {
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
    } else {
      digitalWrite(FAN_IN1, LOW);
      digitalWrite(FAN_IN2, LOW);
    }
  }
}




