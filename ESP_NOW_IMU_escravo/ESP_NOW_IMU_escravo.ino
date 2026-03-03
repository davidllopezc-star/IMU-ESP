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

// ====== DESTINO: MAC del MAESTRO (STA) ======
uint8_t masterAddress[] = {0x94,0xE6,0x86,0x3B,0x9B,0xB4};

// ====== CANAL del AP donde está el maestro ======
#define CANAL_WIFI 9

// ====== Payload (DEBE SER IGUAL al del maestro) ======
typedef struct __attribute__((packed)) {
  uint32_t t_ms;     // <-- NUEVO: timestamp en ms (del esclavo)
  float GIRO_X1;
  float GIRO_Y1;
  float GIRO_Z1;
  float ACEL_X1;
  float ACEL_Y1;
  float ACEL_Z1;
} parametros_t;

parametros_t IMUData;

// ====== Callback envío (IDF5 vs IDF4) ======
#if ESP_IDF_VERSION_MAJOR >= 5
void OnSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  Serial.print("Send message status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent Successfully" : "Sent Failed");
}
#else
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  Serial.print("Send message status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent Successfully" : "Sent Failed");
}
#endif

// ====== MPU/DMP ======
MPU6050 mpu;

float fator_giro = 16.4;          // 2000 dps
float fator_aceleracao = 8192.0;  // 4g

long tempo_anterior_send = 0;
long intervalo_send = 20;         // 50 Hz

#define OUTPUT_READABLE_REALACCEL
#define INTERRUPT_PIN 2
#define LED_PIN 13

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 giro;
VectorInt16 aaReal;
VectorFloat gravity;

// Lecturas
float faz_leitura_ACEL_X(void) { return (aa.x / fator_aceleracao); }
float faz_leitura_ACEL_Y(void) { return (aa.y / fator_aceleracao); }
float faz_leitura_ACEL_Z(void) { return (aa.z / fator_aceleracao); }
float faz_leitura_GIRO_X(void) { return (giro.x / fator_giro); }
float faz_leitura_GIRO_Y(void) { return (giro.y / fator_giro); }
float faz_leitura_GIRO_Z(void) { return (giro.z / fator_giro); }

// Interrupt
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void setup() {
  Serial.begin(115200);
  delay(200);

  // ===== WiFi STA (solo para fijar canal/radio) =====
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  // Forzar canal
  esp_err_t cherr = esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);
  Serial.print("set_channel err="); Serial.print((int)cherr);
  Serial.print(" "); Serial.println(esp_err_to_name(cherr));

  // Leer canal real
  uint8_t primary = 0;
  wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primary, &second);
  Serial.print("SLAVE channel (real): "); Serial.println(primary);

  Serial.print("SLAVE MAC: "); Serial.println(WiFi.macAddress());

  // ===== ESP-NOW init =====
  esp_err_t e = esp_now_init();
  Serial.print("esp_now_init: "); Serial.print((int)e);
  Serial.print(" "); Serial.println(esp_err_to_name(e));
  if (e != ESP_OK) return;

  esp_now_register_send_cb(OnSent);

  // Si el peer ya existe, borrarlo
  if (esp_now_is_peer_exist(masterAddress)) {
    Serial.println("Peer existed -> deleting...");
    esp_err_t de = esp_now_del_peer(masterAddress);
    Serial.print("del_peer: "); Serial.print((int)de);
    Serial.print(" "); Serial.println(esp_err_to_name(de));
  }

  // Agregar peer (maestro)
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, masterAddress, 6);
  peer.channel = CANAL_WIFI;
  peer.encrypt = false;
#if ESP_IDF_VERSION_MAJOR >= 5
  peer.ifidx = WIFI_IF_STA;
#endif

  esp_err_t ae = esp_now_add_peer(&peer);
  Serial.print("add_peer: "); Serial.print((int)ae);
  Serial.print(" "); Serial.println(esp_err_to_name(ae));
  if (ae != ESP_OK) return;

  Serial.print("peer exist now? ");
  Serial.println(esp_now_is_peer_exist(masterAddress) ? "YES" : "NO");

  // ===== I2C =====
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(100000); // robusto en cables largos
#endif

  while (!Serial);

  // ===== MPU init =====
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAIL"));

  delay(2000);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // offsets (los tuyos)
  mpu.setXGyroOffset(55.0);
  mpu.setYGyroOffset(-21.0);
  mpu.setZGyroOffset(-11.0);
  mpu.setXAccelOffset(-960);
  mpu.setYAccelOffset(-1535.0);
  mpu.setZAccelOffset(3316.0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
#ifdef OUTPUT_READABLE_REALACCEL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&giro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    IMUData.GIRO_X1 = faz_leitura_GIRO_X();
    IMUData.GIRO_Y1 = faz_leitura_GIRO_Y();
    IMUData.GIRO_Z1 = faz_leitura_GIRO_Z();
    IMUData.ACEL_X1 = faz_leitura_ACEL_X();
    IMUData.ACEL_Y1 = faz_leitura_ACEL_Y();
    IMUData.ACEL_Z1 = faz_leitura_ACEL_Z();
#endif

    unsigned long tempo_atual = millis();
    if (tempo_atual - tempo_anterior_send > intervalo_send) {
      tempo_anterior_send = tempo_atual;

      // ====== NUEVO: timestamp dentro del paquete ======
      IMUData.t_ms = tempo_atual;

      esp_err_t result = esp_now_send(masterAddress, (uint8_t*)&IMUData, sizeof(IMUData));
      if (result != ESP_OK) {
        Serial.print("esp_now_send ERR = ");
        Serial.print((int)result);
        Serial.print(" ");
        Serial.println(esp_err_to_name(result));
      }
    }
  }
}
