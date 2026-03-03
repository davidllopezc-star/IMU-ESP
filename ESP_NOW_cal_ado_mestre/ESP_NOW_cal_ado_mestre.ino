#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include "esp_idf_version.h"

// I2Cdev and MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// =====================================================
// ===== ESP-NOW: struct + variables ANTES callback =====
// =====================================================
typedef struct parametros {
  float GIRO_X1;
  float GIRO_Y1;
  float GIRO_Z1;
  float ACEL_X1;
  float ACEL_Y1;
  float ACEL_Z1;
} parametros;

volatile parametros IMUData;
volatile bool dados_recebidos = false;

// --- Callback ESP-NOW compatible con Arduino-ESP32 3.x (IDF v5) ---
#if ESP_IDF_VERSION_MAJOR >= 5
void OnRecv(const esp_now_recv_info_t* recv_info, const uint8_t* incomingData, int len) {
  if (!incomingData) return;
  if (len != (int)sizeof(IMUData)) return;
  memcpy((void*)&IMUData, incomingData, sizeof(IMUData));
  dados_recebidos = true;
}
#else
void OnRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  if (!incomingData) return;
  if (len != (int)sizeof(IMUData)) return;
  memcpy((void*)&IMUData, incomingData, sizeof(IMUData));
  dados_recebidos = true;
}
#endif

// =====================================================
// =================== IMU LOCAL (DMP) =================
// =====================================================
MPU6050 mpu;

float fator_giro = 16.4;         // ±2000 dps
float fator_aceleracao = 8192.0; // ±4g

long tempo_anterior_mqtt = 0;
long intervalo_mqtt = 20; // 50 Hz

#define OUTPUT_READABLE_REALACCEL
#define INTERRUPT_PIN 2
#define LED_PIN 13

bool blinkState = false;

// MPU control/status vars
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

// =====================================================
// ======================= MQTT ========================
// =====================================================
#define TOPICO_SUBSCRIBE_LED "topico_liga_desliga_led"
#define ifsp_gru_IMU6050     "ifsp_gru_IMU6050"
#define ID_MQTT              "esp32_mqtt"


const char* SSID = "APTO-402-2.4";
const char* PASSWORD = "Calle144#12-55-402";

const char* BROKER_MQTT = "broker.hivemq.com";
int BROKER_PORT = 1883;

WiFiClient espClient;
PubSubClient MQTT(espClient);

// =====================================================
// =================== Prototypes ======================
// =====================================================
void initWiFi(void);
void initMQTT(void);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT(void);
void reconnectWiFi(void);
void VerificaConexoesWiFIEMQTT(void);

// =====================================================
// =================== Helpers IMU =====================
// =====================================================
float faz_leitura_TIMESTAMP(void) {
  return (millis() / 1000.0);
}
float faz_leitura_ACEL_X(void) { return (aa.x / fator_aceleracao); }
float faz_leitura_ACEL_Y(void) { return (aa.y / fator_aceleracao); }
float faz_leitura_ACEL_Z(void) { return (aa.z / fator_aceleracao); }
float faz_leitura_GIRO_X(void) { return (giro.x / fator_giro); }
float faz_leitura_GIRO_Y(void) { return (giro.y / fator_giro); }
float faz_leitura_GIRO_Z(void) { return (giro.z / fator_giro); }

// =====================================================
// ================== MQTT/WIFI funcs ==================
// =====================================================
void initWiFi(void) {
  delay(10);
  Serial.println("------Conexao WI-FI------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(SSID);
  Serial.println("Aguarde");
  reconnectWiFi();
}

void initMQTT(void) {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setCallback(mqtt_callback);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // opcional
}

void reconnectMQTT(void) {
  while (!MQTT.connected()) {
    Serial.print("* Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado com sucesso ao broker MQTT!");
      MQTT.subscribe(TOPICO_SUBSCRIBE_LED);
    } else {
      Serial.println("Falha ao reconectar no broker. Nova tentativa em 2s");
      delay(2000);
    }
  }
}

void reconnectWiFi(void) {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso na rede ");
  Serial.println(SSID);
  Serial.print("IP obtido: ");
  Serial.println(WiFi.localIP());
  Serial.print("Canal WiFi maestro: ");
  Serial.println(WiFi.channel());
  Serial.print("MAC STA maestro: ");
  Serial.println(WiFi.macAddress());
}

void VerificaConexoesWiFIEMQTT(void) {
  if (!MQTT.connected()) reconnectMQTT();
  reconnectWiFi();
}

// =====================================================
// =============== Interrupt routine DMP ================
// =====================================================
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// =====================================================
// ======================= setup ========================
// =====================================================
void setup() {
  Serial.begin(115200);

  // WiFi
  WiFi.mode(WIFI_AP_STA);

  // I2C
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  delay(2000);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

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

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);

  initWiFi();
  initMQTT();

  // ESP-NOW init + callback (una sola vez, en setup)
  if (esp_now_init() != ESP_OK) {
    Serial.println("There was an error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnRecv);

  Serial.println("Setup maestro completo.");
}

// =====================================================
// ======================== loop ========================
// =====================================================
void loop() {
  char TIMESTAMP_str[12] = {0};
  char ACEL_X_str[12] = {0};
  char ACEL_Y_str[12] = {0};
  char ACEL_Z_str[12] = {0};
  char GIRO_X_str[12] = {0};
  char GIRO_Y_str[12] = {0};
  char GIRO_Z_str[12] = {0};

  char ACEL_X1_str[12] = {0};
  char ACEL_Y1_str[12] = {0};
  char ACEL_Z1_str[12] = {0};
  char GIRO_X1_str[12] = {0};
  char GIRO_Y1_str[12] = {0};
  char GIRO_Z1_str[12] = {0};

  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
#ifdef OUTPUT_READABLE_REALACCEL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&giro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
#endif

    VerificaConexoesWiFIEMQTT();
    MQTT.loop();

    sprintf(TIMESTAMP_str,"%.6f", faz_leitura_TIMESTAMP());
    sprintf(ACEL_X_str,"%.6f", faz_leitura_ACEL_X());
    sprintf(ACEL_Y_str,"%.6f", faz_leitura_ACEL_Y());
    sprintf(ACEL_Z_str,"%.6f", faz_leitura_ACEL_Z());
    sprintf(GIRO_X_str,"%.6f", faz_leitura_GIRO_X());
    sprintf(GIRO_Y_str,"%.6f", faz_leitura_GIRO_Y());
    sprintf(GIRO_Z_str,"%.6f", faz_leitura_GIRO_Z());

    sprintf(ACEL_X1_str,"%.6f", IMUData.ACEL_X1);
    sprintf(ACEL_Y1_str,"%.6f", IMUData.ACEL_Y1);
    sprintf(ACEL_Z1_str,"%.6f", IMUData.ACEL_Z1);
    sprintf(GIRO_X1_str,"%.6f", IMUData.GIRO_X1);
    sprintf(GIRO_Y1_str,"%.6f", IMUData.GIRO_Y1);
    sprintf(GIRO_Z1_str,"%.6f", IMUData.GIRO_Z1);

    unsigned long tempo_atual_mqtt = millis();
    if (dados_recebidos && (tempo_atual_mqtt - tempo_anterior_mqtt > (unsigned long)intervalo_mqtt)) {
      dados_recebidos = false;
      tempo_anterior_mqtt = tempo_atual_mqtt;

      StaticJsonDocument<300> doc;
      char buffer[200];

      JsonArray data = doc.createNestedArray("data");
      data.add(TIMESTAMP_str);
      data.add(GIRO_X_str);
      data.add(GIRO_Y_str);
      data.add(GIRO_Z_str);
      data.add(ACEL_X_str);
      data.add(ACEL_Y_str);
      data.add(ACEL_Z_str);

      data.add(GIRO_X1_str);
      data.add(GIRO_Y1_str);
      data.add(GIRO_Z1_str);
      data.add(ACEL_X1_str);
      data.add(ACEL_Y1_str);
      data.add(ACEL_Z1_str);

      serializeJson(doc, buffer);
      MQTT.publish(ifsp_gru_IMU6050, buffer);
      Serial.println(buffer);

      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}
