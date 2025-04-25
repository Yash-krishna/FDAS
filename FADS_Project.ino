#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// **Pin Definitions**
#define LM35_PIN   PA0  // LM35 Temperature Sensor
#define MQ2_PIN    PA1  // MQ-2 Smoke Sensor
#define BUZZER_PIN PA2  // Buzzer Alert
#define RL 1.0           // Load resistance (1kΩ)

// **LCD I2C (16x4)**
LiquidCrystal_I2C lcd(0x27, 16, 4);

// **GSM (SIM800L)**
SoftwareSerial sim800l(PA9, PA10);  // (RX, TX)

// **GPS (NEO-6M)**
SoftwareSerial gpsSerial(PA3, PA2); // (RX, TX)
TinyGPSPlus gps;

bool smsSent = false;  // Prevent duplicate SMS
float R0 = 10.0;  // MQ-2 Calibration

// **MQ-2 Calibration**
void calibrateMQ2() {
    float sum = 0;
    Serial.println("Calibrating MQ-2... Wait 5 seconds");
    for (int i = 0; i < 100; i++) {
        int sensorValue = analogRead(MQ2_PIN);
        float voltage = sensorValue * (3.3 / 4095.0);
        float RS = ((3.3 - voltage) * RL) / voltage;
        sum += RS;
        delay(50);
    }
    R0 = sum / 100;
    Serial.print("Calibrated R0: ");
    Serial.println(R0);
}

// **Read Smoke Level (PPM)**
float getSmokePPM() {
    int sensorValue = analogRead(MQ2_PIN);
    float voltage = sensorValue * (3.3 / 4095.0);
    float RS = ((3.3 - voltage) * RL) / voltage;
    float ratio = RS / R0;
    
    float A = 1000.0, B = -2.2; // MQ-2 Constants
    float ppm = A * pow(ratio, B);
    return ppm;
}

// **Get GPS Coordinates**
String getGPSLocation() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isValid()) {
        return "https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    } else {
        return "GPS not available";
    }
}

// **Send Fire Alert SMS**
void sendSMS() {
    Serial.println("Sending SMS...");

    String location = getGPSLocation();
    String message = "FIRE ALERT! Location: " + location;

    sim800l.println("AT+CMGF=1");  
    delay(1000);
    sim800l.println("AT+CMGS=\"+1234567890\"");  // Replace with your number
    delay(1000);
    sim800l.print(message);
    delay(100);
    sim800l.write(26);  // Send SMS (Ctrl+Z)
    delay(5000);

    Serial.println("SMS Sent!");
}

void setup() {
    Serial.begin(115200);
    sim800l.begin(9600);
    gpsSerial.begin(9600);
    
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); 

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("FDAS Initializing...");
    delay(2000);
    lcd.clear();

    calibrateMQ2();
}

void loop() {
    // Read Temperature from LM35
    int lm35Raw = analogRead(LM35_PIN);
    float voltage = (lm35Raw * 3.3) / 4095.0;
    float temperature = voltage * 100.0;

    // Read Smoke Level from MQ-2
    float smokePPM = getSmokePPM();

    // Display Data on LCD
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C  ");

    lcd.setCursor(0, 1);
    lcd.print("Smoke: ");
    lcd.print(smokePPM);
    lcd.print(" PPM  ");

    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print("C | Smoke: ");
    Serial.print(smokePPM);
    Serial.println(" PPM");

    // Fire Alert Condition
    if (smokePPM > 50) {  
        lcd.setCursor(0, 2);
        lcd.print("ALERT! High Smoke");

        digitalWrite(BUZZER_PIN, HIGH);

        if (!smsSent) {
            sendSMS();
            smsSent = true;
        }
    } else {
        digitalWrite(BUZZER_PIN, LOW);
        lcd.setCursor(0, 2);
        lcd.print("                  ");
        smsSent = false;
    }

    delay(1000);
}
