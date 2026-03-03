#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include "esp_idf_version.h"

#define TOPICO_PUB "ifsp_gru_IMU6050"
#define ID_MQTT "esp32_mqtt"

const char* SSID = "APTO-402-2.4";
const char* PASSWORD = "Calle144#12-55-402";
const char* BROKER_MQTT = "broker.hivemq.com";
int BROKER_PORT = 1883;

WiFiClient espClient;
PubSubClient MQTT(espClient);

// ====== Payload (DEBE SER IGUAL al del esclavo) ======
typedef struct __attribute__((packed)) {
  uint32_t t_ms;     // <-- NUEVO: timestamp del esclavo (ms)
  float GIRO_X1;
  float GIRO_Y1;
  float GIRO_Z1;
  float ACEL_X1;
  float ACEL_Y1;
  float ACEL_Z1;
} parametros_t;

volatile parametros_t IMUData;
volatile bool dados_recebidos = false;

// ====== RX callback (IDF5 vs IDF4) ======
#if ESP_IDF_VERSION_MAJOR >= 5
void OnRecv(const esp_now_recv_info_t* recv_info, const uint8_t* incomingData, int len) {
  (void)recv_info;
  if (!incomingData) return;
  if (len != (int)sizeof(parametros_t)) return;
  memcpy((void*)&IMUData, incomingData, sizeof(parametros_t));
  dados_recebidos = true;
}
#else
void OnRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  (void)mac;
  if (!incomingData) return;
  if (len != (int)sizeof(parametros_t)) return;
  memcpy((void*)&IMUData, incomingData, sizeof(parametros_t));
  dados_recebidos = true;
}
#endif

static void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  Serial.println("------Conexao WI-FI------");
  Serial.print("Conectando-se na rede: ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado. IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Canal WiFi maestro: ");
  Serial.println(WiFi.channel());
  Serial.print("MAC STA maestro: ");
  Serial.println(WiFi.macAddress());
}

static void initMQTT() {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setBufferSize(512);
  MQTT.setKeepAlive(30);
}

static void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.print("* Tentando MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT)) {
      Serial.println("MQTT OK");
    } else {
      Serial.println("MQTT FAIL -> retry 2s");
      delay(2000);
    }
  }
}

uint32_t rx_count = 0;
uint32_t last_stats = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  reconnectWiFi();
  initMQTT();

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAIL");
    return;
  }
  Serial.println("esp_now_init OK");

  esp_now_register_recv_cb(OnRecv);
  Serial.println("ESP-NOW RX listo.");
}

void loop() {
  reconnectWiFi();

  if (!MQTT.connected()) reconnectMQTT();
  MQTT.loop();

  if (dados_recebidos) {
    parametros_t local;

    noInterrupts();
    memcpy(&local, (const void*)&IMUData, sizeof(local));
    dados_recebidos = false;
    interrupts();

    rx_count++;

    JsonDocument doc;
    doc["src"] = "slave1";
    doc["t_ms"] = local.t_ms;   // <-- NUEVO: timestamp recibido del esclavo

    JsonArray data = doc["data"].to<JsonArray>();
    data.add(local.GIRO_X1);
    data.add(local.GIRO_Y1);
    data.add(local.GIRO_Z1);
    data.add(local.ACEL_X1);
    data.add(local.ACEL_Y1);
    data.add(local.ACEL_Z1);

    char buffer[512];
    size_t n = serializeJson(doc, buffer, sizeof(buffer));
    MQTT.publish(TOPICO_PUB, buffer, n);
  }

  uint32_t now = millis();
  if (now - last_stats >= 1000) {
    last_stats = now;
    Serial.print("RX/s = ");
    Serial.println(rx_count);
    rx_count = 0;
  }
}
