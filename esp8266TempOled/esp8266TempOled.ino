#define DLZP_SERIAL false
#if !DLZP_SERIAL
#define Serial _dlzp_e
#endif

#define LED_ON LOW
#define LED_OFF HIGH

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// IR
// // // // // // // // // // // // // // // // // // // // // // // // // // //
#include <IRremote.hpp>

#define IR_PIN



// // // // // // // // // // // // // // // // // // // // // // // // // // //
// Oled Graphics
// // // // // // // // // // // // // // // // // // // // // // // // // // //
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define FONT_HEIGHT 8

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);

inline void displayReset() {
  display.clearDisplay();
  display.setCursor(0, 0);
}

#define DISPLAY_TEMP_START_Y (FONT_HEIGHT * 2)
#define DISPLAY_TEMP_HEIGHT (FONT_HEIGHT * 3)
#define DISPLAY_TEMP_END_Y (DISPLAY_TEMP_START_Y + DISPLAY_TEMP_HEIGHT)

inline void displayPrintTemp() {
  display.setTextSize(3);
  display.setCursor(1, DISPLAY_TEMP_START_Y);
  display.fillRect(1, DISPLAY_TEMP_START_Y, SCREEN_WIDTH, DISPLAY_TEMP_HEIGHT, BLACK);
  display.print((int)(lastTemp));
  display.print('.');
  display.print(((int)(lastTemp * 100)) - (((int)lastTemp) * 100));
  display.write(248);
  display.println("F");
  display.display();
  display.setTextSize(1);

  // TODO graph values over time
}

#define DISPLAY_IR_START_Y DISPLAY_TEMP_END_Y
#define DISPLAY_IR_HEIGHT FONT_HEIGHT
#define DISPLAY_IR_END_Y (DISPLAY_IR_START_Y + FONT_HEIGHT)

inline void displayIR() {
  display.setCursor(0, DISPLAY_IR_START_Y);
  display.fillRect(0, DISPLAY_IR_START_Y, SCREEN_WIDTH, DISPLAY_IR_HEIGHT, BLACK);
  display.print((int)(lastTemp));
  display.print('.');
  display.print(((int)(lastTemp * 100)) - (((int)lastTemp) * 100));
  display.write(248);
  display.println("F");
  display.display();
  IRRawDataType 

  // TODO graph values over time
}

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// Temperature Sampling
// // // // // // // // // // // // // // // // // // // // // // // // // // //
#define THERM_ANALOG A0
#define THERM_RES 10000
#define TEMP_DELAY 100

unsigned long tempTimer = 0;
unsigned int tempSamples = 0;
unsigned long tempSum = 0;
double lastTemp = 0;

inline void sampleTemp() {
  if (millis() - tempTimer <= TEMP_DELAY) {
    return;
  }
  tempTimer = millis();

  tempSamples++;
  tempSum += analogRead(THERM_ANALOG);
}

inline void calculateAverageTemp() {
  // Value not used, other examples had it.
  // float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

  // Verbose description of optimized lines below.
  // auto const analogValue = analogRead(THERM_ANALOG);
  // float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741;
  // auto const thermRes = THERM_RES * (1024 / (float)analogValue - 1);
  // auto const thermResLog = log(thermRes);
  // auto const tempKelvin =
  //     (1.0 / (c1 + (c2*thermResLog) + (c3 * thermResLog * thermResLog *
  //     thermResLog)));
  // auto const tempCelsius = tempKelvin - 273.15;
  // auto const tempFahrenheit = (tempCelsius * 1.8) + 32.0;

  // Optimized:
  // auto value = log(10240000 / analogValue - 10000);
  // value =
  //     (1.8 / (0.001129148 +
  //             (value * (0.000234125 + (value * value * 0.0000000876741)))))
  //             -
  //     459.67;
  if (tempSamples == 0 || tempSum == 0) {
    return;
  }

  lastTemp = log(10240000 / (tempSum / tempSamples) - 10000);
  lastTemp =
      (1.8 / (0.001129148 + (lastTemp * (0.000234125 + (lastTemp * lastTemp *
                                                        0.0000000876741))))) -
      459.67;

  tempSum = 0;
  tempSamples = 0;

#if DLZP_SERIAL
  Serial.println((int)(lastTemp * 100));
#endif
  displayPrintTemp();
}

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// WiFi Transmission
// // // // // // // // // // // // // // // // // // // // // // // // // // //
#include <ESP8266WiFi.h>
#include "secrets.hpp"

#define CONNECT_DELAY 10000
#define TRANSMIT_DELAY 10000

bool wasConnected = false;
unsigned long connectTimer = 0;
unsigned long transmitTimer = 0;

inline void transmit() {
  WiFiClient client;

  digitalWrite(LED_BUILTIN, LED_ON);
  if (!client.connect(SERVER_IP, SERVER_PORT)) {
#if DLZP_SERIAL
    Serial.println("Could not connect to server. Will skip this message.");
#endif
    digitalWrite(LED_BUILTIN, LED_OFF);
    return;
  }

  char buffer[10];
  snprintf(buffer, 10, "temp:%d", (int)(lastTemp * 100));
  client.print(buffer);
  client.flush();

  bool receivedData = false;
  while (client.connected()) {
    while (client.available()) {
      receivedData = true;
      char const c = client.read();
#if DLZP_SERIAL
      Serial.write(c);
#endif
    }
  }
#if DLZP_SERIAL
  if (receivedData) {
    Serial.println();
  }
  // Serial.println("Client now disconnected.");
#endif
  client.stop();

  digitalWrite(LED_BUILTIN, LED_OFF);
}

inline void checkAndTransmit() {
  if (millis() - transmitTimer > TRANSMIT_DELAY) {
    calculateAverageTemp();
    transmit();

    // Always wait a full TRANSMIT_DELAY between attempts.
    transmitTimer = millis();
  }
}

inline bool connectToWifi() {
  // Leave early if WiFi is connected.
  int wifiStatus = WiFi.status();
  if (wifiStatus == WL_CONNECTED) {
    if (!wasConnected) {
      wasConnected = true;
#if DLZP_SERIAL
      Serial.print("Connected with IP ");
      Serial.println(WiFi.localIP());
#endif
      displayReset();
      display.print("Connected ");
      display.println(WiFi.localIP());
      display.display();
    }
    return true;
  } else if (wasConnected) {
    wasConnected = false;
#if DLZP_SERIAL
    Serial.print("Lost WiFi connection: ");
    Serial.println(wifiStatus);
#endif
    displayReset();
    display.print("Lost connection ");
    display.println(wifiStatus);
    display.display();
  }

  if (wifiStatus == WL_DISCONNECTED) {
    // Busy trying to connect.
    return false;
  }

  // Leave early if too soon to try WiFi connection again.
  if ((connectTimer != 0) && (millis() - connectTimer <= CONNECT_DELAY)) {
    return false;
  }
  connectTimer = millis();

  // Connect to WiFi.
  digitalWrite(LED_BUILTIN, LED_ON);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  return false;
}

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// Setup and Loop
// // // // // // // // // // // // // // // // // // // // // // // // // // //

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial init
#if DLZP_SERIAL
  Serial.begin(9600);
  while (!Serial)
    ;
#endif

  // Oled init
  while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
#if DLZP_SERIAL
    Serial.println("SSD1306 allocation failed");
#endif
  }

  display.cp437(true);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  displayReset();
  display.println("Connecting to WiFi...");
  display.display();

  // Wi-Fi init
  WiFi.mode(WIFI_STA);

  // IR init
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  if (connectToWifi()) {
    checkAndTransmit();
    sampleTemp();
  }

  if(IrReceiver.decode()) {

  }
}