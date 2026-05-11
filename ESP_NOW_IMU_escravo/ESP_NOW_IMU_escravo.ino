#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include "esp_idf_version.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// ====== MAC STA del maestro ======
// Usa la MAC que imprime el maestro: MAC STA maestro: XX:XX:XX:XX:XX:XX
uint8_t masterAddress[] = {0x94, 0xE6, 0x86, 0x3B, 0x9B, 0xB4};

// ====== Canal WiFi del maestro ======
// Usa el canal que imprime el maestro
#define CANAL_WIFI 4

// ====== Payload: DEBE coincidir con el maestro ======
typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float GIRO_X1;
  float GIRO_Y1;
  float GIRO_Z1;
  float ACEL_X1;
  float ACEL_Y1;
  float ACEL_Z1;
} parametros_t;

parametros_t IMUData;

// Evita llenar la cola interna de ESP-NOW
volatile bool espnow_busy = false;

// ====== Callback envío ======
#if ESP_IDF_VERSION_MAJOR >= 5
void OnSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  espnow_busy = false;

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  }
}
#else
void OnSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  espnow_busy = false;

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  }
}
#endif

// ====== MPU/DMP ======
MPU6050 mpu;

float fator_giro = 16.4;          // ±2000 dps
float fator_aceleracao = 8192.0;  // ±4g

unsigned long tempo_anterior_send = 0;
const unsigned long intervalo_send = 50;  // 20 Hz temporal para probar estabilidad

#define INTERRUPT_PIN 2
#define LED_PIN 2

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 giro;
VectorInt16 aaReal;
VectorFloat gravity;

float faz_leitura_ACEL_X(void) { return (aa.x / fator_aceleracao); }
float faz_leitura_ACEL_Y(void) { return (aa.y / fator_aceleracao); }
float faz_leitura_ACEL_Z(void) { return (aa.z / fator_aceleracao); }

float faz_leitura_GIRO_X(void) { return (giro.x / fator_giro); }
float faz_leitura_GIRO_Y(void) { return (giro.y / fator_giro); }
float faz_leitura_GIRO_Z(void) { return (giro.z / fator_giro); }

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ===== WiFi STA solo para ESP-NOW =====
// ===== WiFi STA solo para ESP-NOW =====
WiFi.mode(WIFI_OFF);
delay(100);

WiFi.mode(WIFI_STA);
delay(100);

esp_wifi_start();
delay(100);

WiFi.setSleep(false);

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

delay(100);

Serial.print("MAC STA esclavo: ");
Serial.println(WiFi.macAddress());

  // Forzar canal igual al maestro
  esp_err_t cherr = esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);
  Serial.print("set_channel: ");
  Serial.print((int)cherr);
  Serial.print(" ");
  Serial.println(esp_err_to_name(cherr));

  uint8_t primary = 0;
  wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primary, &second);

  Serial.print("Canal WiFi esclavo real: ");
  Serial.println(primary);


  // ===== ESP-NOW =====
  esp_err_t e = esp_now_init();
  Serial.print("esp_now_init: ");
  Serial.print((int)e);
  Serial.print(" ");
  Serial.println(esp_err_to_name(e));

  if (e != ESP_OK) return;

  esp_now_register_send_cb(OnSent);

  if (esp_now_is_peer_exist(masterAddress)) {
    esp_now_del_peer(masterAddress);
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, masterAddress, 6);
  peer.channel = CANAL_WIFI;
  peer.encrypt = false;

#if ESP_IDF_VERSION_MAJOR >= 5
  peer.ifidx = WIFI_IF_STA;
#endif

  esp_err_t ae = esp_now_add_peer(&peer);
  Serial.print("add_peer: ");
  Serial.print((int)ae);
  Serial.print(" ");
  Serial.println(esp_err_to_name(ae));

  if (ae != ESP_OK) return;

  Serial.print("peer exist now? ");
  Serial.println(esp_now_is_peer_exist(masterAddress) ? "YES" : "NO");

  // ===== I2C =====
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(100000);
#endif

  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");

  delay(500);

  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // Offsets
  mpu.setXGyroOffset(55.0);
  mpu.setYGyroOffset(-21.0);
  mpu.setZGyroOffset(-11.0);
  mpu.setXAccelOffset(-960);
  mpu.setYAccelOffset(-1535.0);
  mpu.setZAccelOffset(3316.0);

  // Rangos coherentes con factores
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    dmpReady = true;
    Serial.println("DMP ready.");
  } else {
    Serial.print("DMP init failed: ");
    Serial.println(devStatus);
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (!dmpReady) return;

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGyro(&giro, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  IMUData.t_ms = millis();

  IMUData.GIRO_X1 = faz_leitura_GIRO_X();
  IMUData.GIRO_Y1 = faz_leitura_GIRO_Y();
  IMUData.GIRO_Z1 = faz_leitura_GIRO_Z();

  // Igual que tu versión original: aceleración cruda con gravedad
  IMUData.ACEL_X1 = faz_leitura_ACEL_X();
  IMUData.ACEL_Y1 = faz_leitura_ACEL_Y();
  IMUData.ACEL_Z1 = faz_leitura_ACEL_Z();

  unsigned long tempo_atual = millis();

  if (!espnow_busy && (tempo_atual - tempo_anterior_send >= intervalo_send)) {
    tempo_anterior_send = tempo_atual;

    espnow_busy = true;

    esp_err_t result = esp_now_send(
      masterAddress,
      (uint8_t*)&IMUData,
      sizeof(IMUData)
    );

    if (result != ESP_OK) {
      espnow_busy = false;

      Serial.print("esp_now_send ERR = ");
      Serial.print((int)result);
      Serial.print(" ");
      Serial.println(esp_err_to_name(result));
    }

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}