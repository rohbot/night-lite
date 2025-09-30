#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// ---------- LED config ----------
#define LED_PIN 7
#define NUM_LEDS 1
#define LED_TYPE WS2812
#define COLOR_ORDER RGB

CRGB leds[NUM_LEDS];

bool ledState = true;         // LED starts ON
bool lastButtonState = HIGH;  // because of pullup
unsigned long lastDebounceTime = 0;
unsigned long lastPrint = 0;

// ---------- Button config ----------
#define BUTTON_PIN 9
#define DEBOUNCE_MS 500  // debounce time in ms
#define PRINT_MS 2000    // print time
// ---------- BME280 config ----------
#define SDA_PIN 5
#define SCL_PIN 6

#define COLOUR CRGB(255, 120, 0)

Adafruit_BME280 bme;
bool bme_ok = false;

void setup() {
  // Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Zero: WS2812 + Button + BME280");

  // LED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(80);
  leds[0] = COLOUR;  // start ON
  FastLED.show();

  // Button with internal pullup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // I2C init with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // BME280 init at 0x77
  if (bme.begin(0x77, &Wire)) {
    Serial.println("BME280 detected at 0x77!");
    bme_ok = true;
  } else {
    Serial.println("Could not find BME280 sensor at 0x77!");
  }
  lastDebounceTime = millis();
}

void loop() {
  // ---- Button debounce + LED toggle ----
  int reading = digitalRead(BUTTON_PIN);

  // Serial.println("here");
  if (reading == LOW) {
    
    if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
      // Button pressed
      Serial.println("Button pressed!");

      ledState = !ledState;  // toggle LED state
      if (ledState) {
        leds[0] = COLOUR;
      } else {
        leds[0] = CRGB::Black;
      }
      FastLED.show();
      lastDebounceTime = millis();
    }
  }




  if ((millis() - lastPrint) > PRINT_MS) {
    // ---- BME280 readings ----
    if (bme_ok) {
      float temperature = bme.readTemperature();  // °C
      float humidity = bme.readHumidity();        // %

      Serial.print("Temp: ");
      Serial.print(temperature);
      Serial.print(" °C  |  Humidity: ");
      Serial.print(humidity);
      Serial.print(" % | Reading: ");
      Serial.println(reading);
      Serial.println(lastButtonState);
      lastPrint = millis();
    }
  }
}
