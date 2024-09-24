#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include <math.h>

#include "secrets.hpp"

#define SERIAL false
#if !SERIAL
#define Serial _dlzp_e
#endif

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// Bump Detection
// // // // // // // // // // // // // // // // // // // // // // // // // // //
#define BUMP_SENSITIVITY 0.1

float lastX = 0, lastY = 0, lastZ = 0;
float bumpAmount = 0;
bool ledStatus = false;
unsigned long lastBump = 0;

inline void checkForBump() {
  if (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);

    bumpAmount = abs(lastX - x) + abs(lastY - y) + abs(lastZ - z);
    bool const bumpDetected = (bumpAmount > BUMP_SENSITIVITY);

    if (bumpDetected) {
      lastX = x;
      lastY = y;
      lastZ = z;
      lastBump = millis();
    }

    if (bumpDetected != ledStatus) {
      digitalWrite(LED_BUILTIN, bumpDetected ? HIGH : LOW);
      ledStatus = bumpDetected;
    }
  }
}

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// Temperature Sampling
// // // // // // // // // // // // // // // // // // // // // // // // // // //
#define THERM_ANALOG PIN_A0
#define THERM_RES 10000

unsigned int tempSamples = 0;
unsigned long tempSum = 0;
double lastTemp = 0;

inline void sampleTemp() {
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
}

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// WiFi Transmission
// // // // // // // // // // // // // // // // // // // // // // // // // // //
#define TRANSMIT_DELAY 10000

unsigned long transmitTimer = 0;

inline void transmit() {
  WiFiClient client;

  digitalWrite(LED_BUILTIN, HIGH);
  if (!client.connect(SERVER_IP, SERVER_PORT)) {
#if SERIAL
    Serial.println("Could not connect to server. Will skip this message.");
#endif
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  char buffer[10];
  snprintf(buffer, 10, "temp:%d", (int)(lastTemp * 100));
  client.print(buffer);
  client.flush();

  while (client.connected()) {
    while (client.available()) {
      char const c = client.read();
#if SERIAL
      Serial.write(c);
#endif
    }
  }
#if SERIAL
  Serial.println("Client now disconnected.");
#endif
  client.stop();

  digitalWrite(LED_BUILTIN, LOW);
}

inline void checkAndTransmit() {
  if (millis() - transmitTimer > TRANSMIT_DELAY) {
    calculateAverageTemp();
    transmit();

    // Always wait a full TRANSMIT_DELAY between attempts.
    transmitTimer = millis();
  }
}

inline void connectToWifi() {
  int wifiStatus = WiFi.status();
  if (wifiStatus == WL_CONNECTED) {
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  // Loop connect to WiFi.
  while (true) {
    wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
    if (wifiStatus == WL_CONNECTED) {
      break;
    }

#if SERIAL
    Serial.print(
        "Couldn't connect to WiFi, will try again in 10 seconds. Code: ");
    Serial.println(wifiStatus);
#endif

    delay(10000);
  }

#if SERIAL
  Serial.print("Connected to WiFi: ");
  Serial.println(WiFi.localIP());
#endif

  digitalWrite(LED_BUILTIN, LOW);
}

// // // // // // // // // // // // // // // // // // // // // // // // // // //
// Setup and Loop
// // // // // // // // // // // // // // // // // // // // // // // // // // //

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(THERM_ANALOG, INPUT);

  // Initialize modules.
#if SERIAL
  Serial.begin(9600);
  while (!Serial)
    ;

  if (!IMU.begin()) {
    Serial.println("IMU failed to begin!");
    while (1)
      ;
  }

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not found!");
    while (1)
      ;
  }

  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
#else
  while (!IMU.begin() || WiFi.status() == WL_NO_MODULE)
    ;
#endif
}

void loop() {
  connectToWifi();
  checkForBump();
  sampleTemp();
  checkAndTransmit();
}