
#include <Arduino.h>

// Pin sensor sesuai konfigurasi robot
const uint8_t SENSOR_PINS[8] = {
  13, 14, 27, 26, 25, 33, 32, 35
};

const int SENSOR_COUNT = 8;

void setup() {
  Serial.begin(115200);

  // Resolusi ADC ESP32: 12-bit (0-4095)
  analogReadResolution(12);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  Serial.println("=== TES SENSOR LINE FOLLOWER ===");
  Serial.println("Arahkan sensor ke bidang putih atau hitam.");
  Serial.println("Baud rate Serial Monitor: 115200");
  Serial.println();
}

void loop() {
  // Header kolom
  Serial.print("S1\tS2\tS3\tS4\t");
  Serial.println("S5\tS6\tS7\tS8");

  // Baca kedelapan sensor
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int value = analogRead(SENSOR_PINS[i]);

    Serial.print(value);

    if (i < SENSOR_COUNT - 1) {
      Serial.print('\t');
    }
  }

  Serial.println();
  delay(500);
}
