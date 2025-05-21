#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include "AdafruitIO_WiFi.h"
#include <ServeSmartClassifier_inferencing.h>

// Adafruit IO bilgileri
#define IO_USERNAME  "melikesraoz"
#define IO_KEY       "****"

// WiFi bilgileri
#define WIFI_SSID     "Test"
#define WIFI_PASS     "test1234"

// Adafruit IO bağlantısı
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

// Feed tanımları
AdafruitIO_Feed *feed_acc_total = io.feed("acc_total");
AdafruitIO_Feed *feed_velocity_total = io.feed("velocity_total");
AdafruitIO_Feed *feed_prediction = io.feed("prediction");
AdafruitIO_Feed *feed_comment = io.feed("comment");

MPU6050 mpu;

// Hızlar (m/s)
float vx = 0, vy = 0, vz = 0;
const float dt = 0.2; // delay 2000ms = 0.2s

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) Serial.println("MPU6050 hazır!");
  else Serial.println("MPU bağlantı hatası!");

  Serial.print("Adafruit IO'ya bağlanılıyor...");
  io.connect();

  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\n✔ Adafruit IO'ya bağlanıldı!");
}

void loop() {
  io.run();

  Serial.println("----- Yeni döngü başlıyor -----");

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.printf("Ham veri: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n", ax, ay, az, gx, gy, gz);

  // Hareket yoksa sınıflandırma yapma
  float acc_raw = sqrt(ax * ax + ay * ay + az * az);
  if (acc_raw < 500) {
    Serial.println("🔇 Hareket yok, sınıflandırma yapılmadı.");
    feed_prediction->save("idle");
    feed_comment->save("No motion");
    delay(2000);
    return;
  }

  // Edge Impulse verisi
  float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
  features[0] = ax;
  features[1] = ay;
  features[2] = az;
  features[3] = gx;
  features[4] = gy;
  features[5] = gz;

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = [&features](size_t offset, size_t length, float *out_ptr) -> int {
    memcpy(out_ptr, &features[offset], length * sizeof(float));
    return 0;
  };

  ei_impulse_result_t result = {0};
  Serial.println("Model çalıştırılıyor...");
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  Serial.println("Model çalıştırıldı ✅");

  String prediction = "unknown";
  float confidence = 0.0;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > confidence) {
      prediction = String(ei_classifier_inferencing_categories[i]);
      confidence = result.classification[i].value;
    }
  }

  // m/s² cinsinden ivme (az'den gravity çıkarıldı)
  float ax_mps2 = ax / 16384.0 * 9.80665;
  float ay_mps2 = ay / 16384.0 * 9.80665;
  float az_mps2 = (az / 16384.0 - 1.0) * 9.80665;  // gravity düzeltmesi

  // hız güncelle
  vx += ax_mps2 * dt;
  vy += ay_mps2 * dt;
  vz += az_mps2 * dt;

  float acc_total = sqrt(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);
  float velocity_total = sqrt(vx * vx + vy * vy + vz * vz);

  // Yorum üret
  String comment = "Good";
  if (confidence < 0.6) comment = "Low confidence";
  else if (acc_total < 1.2) comment = "Too slow";
  else if (prediction == "idle") comment = "No shot";

  // Çıktılar
  Serial.printf("Prediction: %s (%.2f)\n", prediction.c_str(), confidence);
  Serial.printf("Ivme: %.2f m/s² | Hiz: %.2f m/s | Yorum: %s\n", acc_total, velocity_total, comment.c_str());

  // Verileri Adafruit IO'ya gönder
  feed_acc_total->save(acc_total);
  feed_velocity_total->save(velocity_total);
  feed_prediction->save(prediction);
  feed_comment->save(comment);

  delay(5000); // 2 saniye
}
