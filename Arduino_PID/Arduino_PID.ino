// ==========================================
// LINE FOLLOWER ROBOT - ESP32 (versi diperbaiki)
// ==========================================
// Perubahan dari versi awal:
// 1. Resolusi ADC disamakan (analogReadResolution) & threshold disesuaikan
// 2. Catatan pin ADC2 vs WiFi
// 3. Penanganan saat garis hilang (kembali ke arah terakhir terlihat)
// 4. PID pakai delta waktu (dt) + anti-windup pada integral
// 5. Motor bisa mundur (bidirectional) untuk belokan tajam
// 6. Kalibrasi sensor opsional (calibrate())
// ==========================================

#include <Arduino.h>

// ==========================================
// 1. CLASS SENSOR ARRAY (8 Sensor)
// ==========================================
class SensorArray {
  private:
    uint8_t pins[8];
    int threshold;
    int lastPosition = 0;      // posisi terakhir saat garis masih terdeteksi
    int lastActiveSensors = 0; // jumlah sensor aktif pada pembacaan terakhir

  public:
    // thresholdValue default disesuaikan untuk ADC 12-bit ESP32 (0-4095)
    SensorArray(const uint8_t sensorPins[8], int thresholdValue = 2000) {
      for (int i = 0; i < 8; i++) {
        pins[i] = sensorPins[i];
      }
      threshold = thresholdValue;
    }

    void begin() {
      // Set resolusi ADC secara eksplisit supaya nilai threshold konsisten
      // (default ESP32 sebenarnya sudah 12-bit, tapi ditulis eksplisit
      // supaya jelas dan tidak berubah kalau core/board berbeda).
      analogReadResolution(12);
      for (int i = 0; i < 8; i++) {
        pinMode(pins[i], INPUT);
      }
    }

    void setThreshold(int t) { threshold = t; }

    // Jumlah sensor yang aktif (membaca hitam) pada pemanggilan getPosition()
    // terakhir. 0 berarti garis hilang total; 8 berarti semua sensor aktif
    // (misal permukaan hitam lebar / garis finish).
    int getActiveSensorCount() { return lastActiveSensors; }

    // Kalibrasi sederhana: putar/geser robot di atas garis selama calibrateMs,
    // ambil rata-rata nilai min & max tiap sensor, lalu set threshold di
    // tengah-tengahnya. Panggil sekali di setup() sebelum loop utama, opsional.
    void calibrate(unsigned long calibrateMs = 2000) {
      int minVal[8], maxVal[8];
      for (int i = 0; i < 8; i++) {
        minVal[i] = 4095;
        maxVal[i] = 0;
      }

      unsigned long start = millis();
      while (millis() - start < calibrateMs) {
        for (int i = 0; i < 8; i++) {
          int v = analogRead(pins[i]);
          if (v < minVal[i]) minVal[i] = v;
          if (v > maxVal[i]) maxVal[i] = v;
        }
      }

      long sumMid = 0;
      for (int i = 0; i < 8; i++) {
        sumMid += (minVal[i] + maxVal[i]) / 2;
      }
      threshold = sumMid / 8;
    }

    // Mengembalikan posisi garis (-3500 paling kiri, 0 tengah, 3500 paling kanan).
    // Jika garis hilang (tidak ada sensor aktif), kembalikan posisi ekstrem
    // terakhir supaya robot tetap berbelok mencari garis, bukan jalan lurus.
    int getPosition() {
      long weightedSum = 0;
      int activeSensors = 0;
      static const int weights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

      for (int i = 0; i < 8; i++) {
        int value = analogRead(pins[i]);
        if (value > threshold) {
          weightedSum += weights[i];
          activeSensors++;
        }
      }

      lastActiveSensors = activeSensors;

      if (activeSensors == 0) {
        return (lastPosition < 0) ? -3500 : 3500;
      }

      lastPosition = weightedSum / activeSensors;
      return lastPosition;
    }
};

// ==========================================
// 2. CLASS PID CONTROLLER
// ==========================================
class PIDController {
  private:
    float kp, ki, kd;
    float lastError = 0;
    float integral = 0;
    float integralMax;      // batas anti-windup
    unsigned long lastTime = 0;

  public:
    PIDController(float p, float i, float d, float integralLimit = 2000)
      : kp(p), ki(i), kd(d), integralMax(integralLimit) {}

    float calculate(int setpoint, int currentPosition) {
      unsigned long now = micros();
      float dt = (lastTime == 0) ? 0.01f : (now - lastTime) / 1000000.0f;
      lastTime = now;
      if (dt <= 0) dt = 0.001f; // jaga-jaga pembagian nol

      float error = setpoint - currentPosition;

      integral += error * dt;
      integral = constrain(integral, -integralMax, integralMax); // anti-windup

      float derivative = (error - lastError) / dt;
      lastError = error;

      return (kp * error) + (ki * integral) + (kd * derivative);
    }

    // Panggil ini kalau robot berhenti lama / garis hilang lama,
    // supaya integral & lastError tidak membawa nilai basi.
    void reset() {
      integral = 0;
      lastError = 0;
      lastTime = 0;
    }
};

// ==========================================
// 3. CLASS MOTOR DRIVER
// ==========================================
class MotorDriver {
  private:
    uint8_t pinL1, pinL2, pinPWML;
    uint8_t pinR1, pinR2, pinPWMR;

    void setMotor(int speed, uint8_t pin1, uint8_t pin2, uint8_t pinPWM) {
      speed = constrain(speed, -255, 255);
      if (speed >= 0) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
      } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        speed = -speed;
      }
      analogWrite(pinPWM, speed);
    }

  public:
    MotorDriver(uint8_t l1, uint8_t l2, uint8_t pwml, uint8_t r1, uint8_t r2, uint8_t pwmr)
      : pinL1(l1), pinL2(l2), pinPWML(pwml), pinR1(r1), pinR2(r2), pinPWMR(pwmr) {}

    void begin() {
      pinMode(pinL1, OUTPUT); pinMode(pinL2, OUTPUT); pinMode(pinPWML, OUTPUT);
      pinMode(pinR1, OUTPUT); pinMode(pinR2, OUTPUT); pinMode(pinPWMR, OUTPUT);
    }

    // baseSpeed + correction bisa membuat salah satu roda mundur (nilai negatif)
    // supaya belokan tajam lebih responsif, bukan cuma berhenti.
    void drive(int baseSpeed, float correction) {
      int speedL = baseSpeed - (int)correction;
      int speedR = baseSpeed + (int)correction;
      setMotor(speedL, pinL1, pinL2, pinPWML);
      setMotor(speedR, pinR1, pinR2, pinPWMR);
    }

    void stop() {
      analogWrite(pinPWML, 0);
      analogWrite(pinPWMR, 0);
    }
};

// ==========================================
// INISIALISASI PERANGKAT HARDWARE
// ==========================================
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define lebarlayar 64
#define panjanglayar 128
Adafruit_SSD1306 layar(panjanglayar, lebarlayar, &Wire, -1);
const int panjangKotak[8] = {
  103, 90, 77, 64, 51, 38, 25, 12
};
// CATATAN PENTING (ESP32 + WiFi):
// Pin 25, 26, 27, 14, 12, 13 berada di ADC2, yang TIDAK stabil dipakai
// analogRead() saat WiFi aktif (ESP32 memakai ADC2 untuk WiFi secara
// internal). Kalau proyek ini nanti pakai WiFi/BLE, pindahkan sensor
// ke pin ADC1 (32, 33, 34, 35, 36, 39) atau matikan WiFi saat membaca sensor.
const uint8_t SENSOR_PINS[8] = {13, 14, 27, 26, 25, 33, 32, 35};

SensorArray sensors(SENSOR_PINS, 2000); // threshold awal, bisa dikalibrasi ulang
PIDController pid(1.2, 0.0, 0.5, 2000);
MotorDriver motors(17, 16, 4, 18, 19, 23);

const int BASE_SPEED = 150;

// Batas waktu garis hilang sebelum motor dihentikan (ms)
const unsigned long LINE_LOST_TIMEOUT_MS = 5000;

unsigned long lineLostStartTime = 0; // 0 berarti timer belum berjalan
bool robotStopped = false;           // latch: sekali true, motor berhenti permanen

void setup() {
  Serial.begin(115200);
  sensors.begin();
  motors.begin();
  layar.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  layar.clearDisplay();

  // Opsional: kalibrasi sensor dulu (geser robot melewati garis selama
  // proses ini). Hapus/komentari baris ini kalau threshold manual sudah pas.
  // sensors.calibrate(2000);
}

void loop() {
    layar.clearDisplay();
    layar.setTextColor(SSD1306_WHITE);
    layar.setTextSize(2);
    for (int i = 0; i < 8; i++){
      layar.fillRect(panjangKotak[i], 12, 12, analogRead(SENSOR_PINS[i]) / 102.375, SSD1306_WHITE);
    }
    layar.display();
  // Kalau robot sudah dalam status berhenti (latch), motor tetap mati
  // dan tidak lanjut ke logika PID. Untuk jalan lagi, reset/restart board.
  if (robotStopped) {
    motors.stop();
    return;
  }

  int position = sensors.getPosition();        // 1. Baca posisi garis
  int activeCount = sensors.getActiveSensorCount();

  // Kondisi A: semua sensor aktif -> biasanya penanda garis finish/
  // persimpangan lebar yang menutupi seluruh array sensor.
  if (activeCount == 8) {
    robotStopped = true;
    motors.stop();
    return;
  }

  // Kondisi B: garis hilang total -> hitung sudah berapa lama hilang.
  if (activeCount == 0) {
    if (lineLostStartTime == 0) {
      lineLostStartTime = millis(); // mulai hitung waktu kehilangan garis
    } else if (millis() - lineLostStartTime >= LINE_LOST_TIMEOUT_MS) {
      robotStopped = true;
      motors.stop();
      return;
    }
  } else {
    lineLostStartTime = 0; // garis ditemukan lagi, reset timer
  }

  float outputPID = pid.calculate(0, position); // 2. Hitung PID (setpoint = 0)
  motors.drive(BASE_SPEED, outputPID);          // 3. Jalankan motor

  // Debug (aktifkan saat tuning, nonaktifkan lagi setelah PID pas
  // supaya loop tetap cepat):
  Serial.print("Pos: "); Serial.print(position);
  Serial.print(" | PID: "); Serial.println(outputPID);
}
