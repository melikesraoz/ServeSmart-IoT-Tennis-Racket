#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include "AdafruitIO_WiFi.h"
#include <ServeSmartClassifier_inferencing.h>



#define IO_USERNAME  "melikesraoz"
#define IO_KEY       "****"

#define WIFI_SSID     "Test"
#define WIFI_PASS     "test1234"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

AdafruitIO_Feed *feed_acc_total = io.feed("acc_total");
AdafruitIO_Feed *feed_velocity_total = io.feed("velocity_total");
AdafruitIO_Feed *feed_prediction = io.feed("prediction");
AdafruitIO_Feed *feed_comment = io.feed("comment");

MPU6050 mpu;

float vx = 0, vy = 0, vz = 0;
const float dt = 0.2; 

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

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  
  float acc_raw = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  float gravity_magnitude = 1.0; 
  
  if (abs(acc_raw - gravity_magnitude) < 0.2) {
    Serial.println("🔇 Hareket yok, sınıflandırma yapılmadı.");
    feed_prediction->save("idle");  
    feed_comment->save("Hareketsiz");
    
    vx = 0;
    vy = 0;
    vz = 0;
    
    feed_acc_total->save(0);
    feed_velocity_total->save(0);
    
    delay(2000);
    return;
  }
  
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

  float ax_mps2 = ax_g * 9.80665;
  float ay_mps2 = ay_g * 9.80665;
  float az_mps2 = az_g * 9.80665;
  
  float motion_ax = ax_mps2;
  float motion_ay = ay_mps2;
  float motion_az = az_mps2 - 9.80665;
  
  // Hız hesabı
  vx += motion_ax * dt;
  vy += motion_ay * dt;
  vz += motion_az * dt;
  
  if (abs(acc_raw - gravity_magnitude) < 0.2) {
    vx = 0;
    vy = 0;
    vz = 0;
  }

  float acc_total = sqrt(motion_ax * motion_ax + motion_ay * motion_ay + motion_az * motion_az);
  float velocity_total = sqrt(vx * vx + vy * vy + vz * vz);

  String comment = "Good";
  if (confidence < 0.5) {
    comment = "Tanımlanamayan hareket";
    prediction = "unknown";  
  }
  else if (acc_total < 1.2) comment = "Çok yavaş vuruş";
  else if (acc_total > 4.0) comment = "Çok sert vuruş";
  else comment = "İyi vuruş";

  Serial.printf("Prediction: %s (%.2f)\n", prediction.c_str(), confidence);
  Serial.printf("Ivme: %.2f m/s² | Hiz: %.2f m/s | Yorum: %s\n", acc_total, velocity_total, comment.c_str());

  feed_acc_total->save(acc_total);
  feed_velocity_total->save(velocity_total);
  feed_prediction->save(prediction);
  feed_comment->save(comment);

  delay(5000); 
}
