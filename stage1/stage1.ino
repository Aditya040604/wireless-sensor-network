#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ------------------ Sensor Class ------------------
class DHTSensor {
  private:
    int pin;
    int type;
    DHT dht;
    float temperature;
    float humidity;
    unsigned long lastReadTime;  // track last read
    unsigned long interval;      // how often to read (ms)

  public:
    DHTSensor(int dataPin, int dhtType, unsigned long readInterval) 
      : pin(dataPin), type(dhtType), dht(dataPin, dhtType), interval(readInterval) {}

    void begin() {
      dht.begin();
      lastReadTime = 0;
    }

    void update() {
      unsigned long currentMillis = millis();
      if (currentMillis - lastReadTime >= interval) {
        lastReadTime = currentMillis;

        humidity = dht.readHumidity();
        temperature = dht.readTemperature();

        if (isnan(humidity) || isnan(temperature)) {
          Serial.println("⚠️ Failed to read from DHT sensor!");
          return;
        }

        printData();
      }
    }

    float getTemperature() { return temperature; }
    float getHumidity() { return humidity; }

    void printData() {
      Serial.print("🌡 Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, 💧 Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
    }
};


// ---------------- Soil Moisture Sensor ----------------
class SoilMoistureSensor {
  private:
    int pin;
    unsigned long lastReadTime;
    unsigned long interval;
    int rawValue;       // raw analog value (0 - 1023)
    int moisturePercent; // mapped % value

  public:
    // Constructor
    SoilMoistureSensor(int sensorPin, unsigned long readInterval) {
      pin = sensorPin;
      interval = readInterval;
      lastReadTime = 0;
      rawValue = 0;
      moisturePercent = 0;
    }

    void begin() {
      pinMode(pin, INPUT);
    }

    void update() {
      unsigned long currentMillis = millis();
      if (currentMillis - lastReadTime >= interval) {
        lastReadTime = currentMillis;

        rawValue = analogRead(pin);
        
        // Convert to percentage (calibration might be needed for your sensor)
        // Assuming: 0 = 100% moisture (wet), 1023 = 0% moisture (dry)
        moisturePercent = map(rawValue, 1023, 0, 0, 100);

        printData();
      }
    }

    int getRawValue() {
      return rawValue;
    }

    int getMoisturePercent() {
      return moisturePercent;
    }

    void printData() {
      Serial.print("🌱 Soil Moisture: ");
      Serial.print(moisturePercent);
      Serial.println(" %");
    }
};


// ---------------- MQ-5 Gas Sensor ----------------
class MQ5Sensor {
  private:
    int pin;
    unsigned long lastReadTime;
    unsigned long interval;
    int rawValue;       // raw analog value (0 - 1023)
    int gasLevelPercent; // simple % scale for demo

  public:
    MQ5Sensor(int sensorPin, unsigned long readInterval) {
      pin = sensorPin;
      interval = readInterval;
      lastReadTime = 0;
      rawValue = 0;
      gasLevelPercent = 0;
    }

    void begin() {
      pinMode(pin, INPUT);
    }

    void update() {
      unsigned long currentMillis = millis();
      if (currentMillis - lastReadTime >= interval) {
        lastReadTime = currentMillis;

        rawValue = analogRead(pin);

        // Simple mapping for demo purposes (adjustable)
        gasLevelPercent = map(rawValue, 0, 1023, 0, 100);

        printData();
      }
    }

    int getRawValue() {
      return rawValue;
    }

    int getGasLevelPercent() {
      return gasLevelPercent;
    }

    void printData() {
      Serial.print("🔥 Gas Level: ");
      Serial.print(gasLevelPercent);
      Serial.println(" %");
    }
};


// ---------------- Bluetooth HC-05 Modular Class ----------------
class BluetoothModule {
  private:
    HardwareSerial &btSerial;
    unsigned long lastSendTime;
    unsigned long sendInterval; // ms
    String message;

  public:
    // Constructor
    BluetoothModule(HardwareSerial &serialPort, unsigned long interval = 1000)
      : btSerial(serialPort), sendInterval(interval), lastSendTime(0), message("") {}

    void begin(long baudRate = 9600) {
      btSerial.begin(baudRate);
    }

    // Set the message to send
    void setMessage(String msg) {
      message = msg;
    }

    // Call in loop() to send data non-blocking
    void update() {
      unsigned long currentMillis = millis();
      if (currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;

        if (message.length() > 0) {
          btSerial.println(message);
        }
      }
    }

    // Receive data from phone (optional)
    String receive() {
      String msg = "";
      if (btSerial.available()) {
        msg = btSerial.readStringUntil('\n');
      }
      return msg;
    }

    // Optionally, set a new interval dynamically
    void setInterval(unsigned long interval) {
      sendInterval = interval;
    }
};



class LCDDisplay {
  private:
    LiquidCrystal_I2C lcd;
    unsigned long lastUpdate;
    unsigned long updateInterval;

  public:
    LCDDisplay(uint8_t addr = 0x27, uint8_t cols = 16, uint8_t rows = 2, unsigned long interval = 1000)
      : lcd(addr, cols, rows), lastUpdate(0), updateInterval(interval) {}

    void begin() {
      lcd.init();
      lcd.backlight();
      lcd.clear();
    }

    // Call in loop(), non-blocking update
    void update(float temperature, float humidity, int soil, int gas) {
      unsigned long currentMillis = millis();
      if (currentMillis - lastUpdate >= updateInterval) {
        lastUpdate = currentMillis;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(temperature, 1);
        lcd.print("C H:");
        lcd.print(humidity, 0);
        lcd.print("%");

        lcd.setCursor(0, 1);
        lcd.print("Soil:");
        lcd.print(soil);
        lcd.print("% Gas:");
        lcd.print(gas);
        lcd.print("%");
      }
    }
};







// ------------------ Config ------------------
#define DHTPIN 12
#define DHTTYPE DHT11
#define SOILPIN A0
#define MQ5PIN A1

// Read every 2000 ms (2 sec)
DHTSensor dhtSensor(DHTPIN, DHTTYPE, 2000);
SoilMoistureSensor soilSensor(SOILPIN, 1000);  // Soil sensor on A0, every 1s
MQ5Sensor mq5(MQ5PIN, 1000);  // read every 1 sec
BluetoothModule bt(Serial1, 1000);
LCDDisplay lcdDisplay(0x27, 16, 2, 1000); // update every 1s



// ------------------ Arduino Setup ------------------
void setup() {

  Serial.begin(9600);
  lcdDisplay.begin();
  dhtSensor.begin();
  soilSensor.begin();
  mq5.begin();
  bt.begin(9600);      // HC-05 default baud



}

// ------------------ Arduino Loop ------------------
void loop() {
  dhtSensor.update();
  soilSensor.update();
  mq5.update();
  lcdDisplay.update(dhtSensor.getTemperature(),
                    dhtSensor.getHumidity(),
                    soilSensor.getMoisturePercent(),
                    mq5.getGasLevelPercent());

  // send json via bluetooth
  bt.setMessage(getSensorDataJSON());
  bt.update(); // non-blocking

  
  // 🔹 Here we can later add soilSensor.update(), mq5.update(), etc.
}

String getSensorDataJSON() {
  String json = "{";

  json += "\"temperature\":" + String(dhtSensor.getTemperature()) + ",";
  json += "\"humidity\":" + String(dhtSensor.getHumidity()) + ",";
  json += "\"soil_moisture\":" + String(soilSensor.getMoisturePercent()) + ",";
  json += "\"gas_level\":" + String(mq5.getGasLevelPercent());

  json += "}";

  return json;
}

