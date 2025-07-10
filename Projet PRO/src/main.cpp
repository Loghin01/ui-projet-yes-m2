#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <math.h>

// === CONFIGURATION WIFI ===
const char* ssid = "ESP32_IMU";
const char* password = "12345678";

// === CONFIGURATION MQTT ===
IPAddress mqttServer(192, 168, 4, 2);
const int mqttPort = 1883;
const char* mqttClientID = "ESP32Client";
const char* mqttTopic = "esp32/pitch";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// === CAPTEUR IMU ===
Adafruit_LSM6DSOX sox;

// === Variables ===
float pitch_offset = 0;
float roll_offset = 0;
float filtered_pitch = 0;
float filtered_roll = 0;
const float alpha = 0.1;

void calibrateIMU() {
  sensors_event_t accel, gyro, temp;
  float pitch_sum = 0, roll_sum = 0;
  const int samples = 100;

  Serial.println("Calibration...");
  for (int i = 0; i < samples; i++) {
    sox.getEvent(&accel, &gyro, &temp);

    float pitch = atan2(accel.acceleration.x,
                        sqrt(accel.acceleration.y * accel.acceleration.y +
                             accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
    float roll = atan2(accel.acceleration.y,
                       sqrt(accel.acceleration.x * accel.acceleration.x +
                            accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;

    pitch_sum += pitch;
    roll_sum += roll;
    delay(10);
  }

  pitch_offset = pitch_sum / samples;
  roll_offset = roll_sum / samples;

  Serial.print("Pitch offset: "); Serial.println(pitch_offset);
  Serial.print("Roll offset: "); Serial.println(roll_offset);
}

void setupWiFiAP() {
  WiFi.softAP(ssid, password);
  delay(1000);
  Serial.print("WiFi AP actif. IP ESP32: ");
  Serial.println(WiFi.softAPIP());
}

void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connexion MQTT...");
    if (mqttClient.connect(mqttClientID)) {
      Serial.println("Connecté !");
    } else {
      Serial.print("Échec, code=");
      Serial.print(mqttClient.state());
      Serial.println("... retry dans 5 sec");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup...");

  setupWiFiAP();
  mqttClient.setServer(mqttServer, mqttPort);

  if (!sox.begin_I2C()) {
    Serial.println("Capteur LSM6DSOX non détecté.");
    while (1) delay(10);
  }

  Serial.println("Capteur détecté !");
  calibrateIMU();
}

void loop() {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();

  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // === Orientation (pitch & roll)
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI - pitch_offset;
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI - roll_offset;

  // === Filtrage
  filtered_pitch = alpha * pitch + (1 - alpha) * filtered_pitch;
  filtered_roll  = alpha * roll  + (1 - alpha) * filtered_roll;

  // === Supprimer la gravité projetée
  float pitch_rad = filtered_pitch * PI / 180.0;
  float roll_rad = filtered_roll * PI / 180.0;

  float g = 9.81;
  float gx = -g * sin(pitch_rad);
  float gy = g * sin(roll_rad) * cos(pitch_rad);
  float gz = g * cos(roll_rad) * cos(pitch_rad);

  float ax_net = ax - gx;
  float ay_net = ay - gy;
  float az_net = az - gz;

  // === Envoi MQTT
  char payload[128];
  snprintf(payload, sizeof(payload),
           "[%.2f,%.2f,%.2f,%.2f,%.2f]",
           ax_net, ay_net, az_net, filtered_pitch, filtered_roll);

  // === Affichage série
  Serial.print("ax_net: "); Serial.print(ax_net);
  Serial.print(" | ay_net: "); Serial.print(ay_net);
  Serial.print(" | az_net: "); Serial.print(az_net);
  Serial.print(" | Pitch: "); Serial.print(filtered_pitch);
  Serial.print(" | Roll: "); Serial.println(filtered_roll);

  if (!mqttClient.publish(mqttTopic, payload)) {
    Serial.println("Erreur envoi MQTT !");
  }

  delay(250);  // 4 Hz
}
