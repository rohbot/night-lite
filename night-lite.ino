#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <esp_now.h>

// ---------- LED config ----------
#define LED_PIN 7
#define NUM_LEDS 1
#define LED_TYPE WS2812
#define COLOR_ORDER RGB

CRGB leds[NUM_LEDS];
bool ledState = true;
unsigned long lastDebounceTime = 0;
unsigned long lastPrint = 0;

// ---------- Button config ----------
#define BUTTON_PIN 9
#define DEBOUNCE_MS 500
#define PRINT_MS 2000

// ---------- BME280 config ----------
#define SDA_PIN 5
#define SCL_PIN 6
#define COLOUR CRGB(255, 120, 0)

Adafruit_BME280 bme;
bool bme_ok = false;

// ---------- ESP-NOW broadcast ----------
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t ESPNOW_CHANNEL = 1;

// Our packet: temperature only
typedef struct {
  float temperatureC;
} SensorPacket;

// NEW callback signature for Arduino-ESP32 v3.x (IDF 5.x)
void onEspNowSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.setChannel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return false;
  }
  esp_now_register_send_cb(onEspNowSent);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer!");
    return false;
  }

  Serial.print("ESP-NOW ready. My MAC: ");
  Serial.println(WiFi.macAddress());
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32-S3 Sender: broadcast temperature over ESP-NOW");

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(80);
  leds[0] = COLOUR;
  FastLED.show();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (bme.begin(0x77, &Wire)) {
    Serial.println("BME280 detected at 0x77!");
    bme_ok = true;
  } else {
    Serial.println("Could not find BME280!");
  }

  initEspNow();
  lastDebounceTime = millis();
}

void loop() {
  // Button debounce + LED toggle
  int reading = digitalRead(BUTTON_PIN);
  if (reading == LOW && (millis() - lastDebounceTime) > DEBOUNCE_MS) {
    ledState = !ledState;
    leds[0] = ledState ? COLOUR : CRGB::Black;
    FastLED.show();
    lastDebounceTime = millis();
  }

  // Send temp every PRINT_MS
  if ((millis() - lastPrint) > PRINT_MS) {
    float temperature = NAN;
    if (bme_ok) {
      temperature = bme.readTemperature();
    }
    SensorPacket pkt;
    pkt.temperatureC = temperature;

    esp_err_t res = esp_now_send(BROADCAST_MAC, (uint8_t*)&pkt, sizeof(pkt));

    Serial.print("Broadcast Temp: ");
    Serial.print(temperature);
    Serial.print(" °C | Result: ");
    Serial.println(res == ESP_OK ? "OK" : String("ERR ") + res);

    lastPrint = millis();
  }
}
