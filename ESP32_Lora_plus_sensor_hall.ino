/*
  Sistema IoT ESP32 + FreeRTOS + LoRa E32 + GPS + Hall + ACS712 + Voltaje + Temperatura + SD
  Autor: Andres Perez
*/

#include <Arduino.h>
#include "LoRa_E32.h"
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

// ================= PINES =================

// LoRa E32
#define LORA_RX   16
#define LORA_TX   17
#define LORA_AUX  27
#define LORA_M0   21
#define LORA_M1   14

// GPS
#define GPS_RX 15
#define GPS_TX 2
#define GPS_BAUD 9600

// Sensores
#define SENSOR_HALL_PIN_1 13
#define SENSOR_HALL_PIN_2 22
#define SensorPin_C       4
#define SensorPin_V       12
#define sensorPin_T1      25
#define sensorPin_T2      26

// SD
#define SD_CS_PIN 33

// ADC
#define ADC_RESOLUCION 4096.0
#define ADC_VREF_mV 3300.0

// ================= OBJETOS =================

HardwareSerial GPSSerial(1);
LoRa_E32 e32ttl(&Serial2, LORA_AUX, LORA_M0, LORA_M1);
TinyGPSPlus gps;

// ================= VARIABLES =================

char letra = 's';
char final = 'f';

float voltaje = 0.0;
float corriente = 0.0;
float temperatura_a = 0.0;
float temperatura_b = 0.0;

float lat = 0.0;
float lng = 0.0;
float gpsSpeed = 0.0;

float velocidad_kmh_1 = 0.0;
float velocidad_kmh_2 = 0.0;
float velocidadPromedio_kmh = 0.0;

volatile unsigned long contadorPulsos1 = 0;
volatile unsigned long contadorPulsos2 = 0;

float radioLlanta_mm = 340.0;
const int PULSOS_POR_REVOLUCION_POR_SENSOR = 1;
const int intervaloMedicion_ms = 1000;

// ACS712
const float K_DIV_I = 0.50f;
const float SENS_mV_A_PIN = 66.0f * K_DIV_I;
float I_ZERO_mV = 0.0f;
const float alphaI = 0.22f;

// Divisor voltaje 15k / 1k
const float R1_V = 15000.0f;
const float R2_V = 1000.0f;
const float DIV_GAIN_V = (R1_V + R2_V) / R2_V;
const float alphaV = 0.15f;

// Tareas
TaskHandle_t TareaSensores;
TaskHandle_t TareaSerial;

// ================= INTERRUPCIONES =================

void IRAM_ATTR pulsoDetectado1() {
  contadorPulsos1++;
}

void IRAM_ATTR pulsoDetectado2() {
  contadorPulsos2++;
}

// ================= FUNCIONES AUXILIARES =================

inline float rawToMilliVolts(int raw) {
  return 3300.0f * (raw / 4095.0f);
}

int readAdcMedian(int pin, int samples = 15) {
  samples = constrain(samples, 3, 31);
  int buf[31];

  for (int i = 0; i < samples; i++) {
    buf[i] = analogRead(pin);
    delayMicroseconds(200);
  }

  for (int i = 0; i < samples - 1; i++) {
    for (int j = i + 1; j < samples; j++) {
      if (buf[j] < buf[i]) {
        int t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
      }
    }
  }

  return buf[samples / 2];
}

float calibrarCero_mV() {
  const int N = 120;
  long acc = 0;

  for (int i = 0; i < N; i++) {
    acc += (long)rawToMilliVolts(readAdcMedian(SensorPin_C, 9));
    delay(2);
  }

  return (float)acc / N;
}

void configurarLoRaNormal() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(100);
}

// ================= SENSORES =================

float leerTemperatura(int pin) {
  uint16_t val = analogRead(pin);
  float milliVolt = val * (ADC_VREF_mV / ADC_RESOLUCION);
  return milliVolt / 10.0;
}

void leerTemperaturas() {
  temperatura_a = leerTemperatura(sensorPin_T1);
  temperatura_b = leerTemperatura(sensorPin_T2);
}

void leerCorrienteVoltaje() {
  float mvV = rawToMilliVolts(readAdcMedian(SensorPin_V, 15));
  float Vbat = (mvV / 1000.0f) * DIV_GAIN_V;

  float mvI = rawToMilliVolts(readAdcMedian(SensorPin_C, 15));
  float Iamp = (mvI - I_ZERO_mV) / SENS_mV_A_PIN;

  static bool init = false;
  static float v_f = 0;
  static float i_f = 0;

  if (!init) {
    v_f = Vbat;
    i_f = Iamp;
    init = true;
  }

  v_f += alphaV * (Vbat - v_f);
  i_f += alphaI * (Iamp - i_f);

  voltaje = v_f;
  corriente = i_f;
}

void calcularVelocidad() {
  noInterrupts();
  unsigned long pulsos1 = contadorPulsos1;
  unsigned long pulsos2 = contadorPulsos2;
  contadorPulsos1 = 0;
  contadorPulsos2 = 0;
  interrupts();

  float circunferencia_m = 2 * PI * (radioLlanta_mm / 1000.0);

  velocidad_kmh_1 = 0;
  velocidad_kmh_2 = 0;

  if (pulsos1 > 0) {
    float rev1 = (float)pulsos1 / PULSOS_POR_REVOLUCION_POR_SENSOR;
    float dist1 = rev1 * circunferencia_m;
    velocidad_kmh_1 = (dist1 / (intervaloMedicion_ms / 1000.0)) * 3.6;
  }

  if (pulsos2 > 0) {
    float rev2 = (float)pulsos2 / PULSOS_POR_REVOLUCION_POR_SENSOR;
    float dist2 = rev2 * circunferencia_m;
    velocidad_kmh_2 = (dist2 / (intervaloMedicion_ms / 1000.0)) * 3.6;
  }

  velocidadPromedio_kmh = (velocidad_kmh_1 + velocidad_kmh_2) / 2.0;
}

void leerGPS() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());

    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
      gpsSpeed = gps.speed.kmph();
    }
  }
}

// ================= SD =================
void guardarEnSD() {

  if (SD.cardType() == CARD_NONE) {
    Serial.println("ERROR: tarjeta SD no detectada");
    return;
  }

  File archivo = SD.open("/mediciones.csv", FILE_APPEND);

  if (!archivo) {
    Serial.println("ERROR: no se pudo abrir /mediciones.csv");
    return;
  }

  archivo.print(millis() / 1000.0, 2);
  archivo.print(",");

  archivo.print(velocidad_kmh_1, 2);
  archivo.print(",");

  archivo.print(velocidad_kmh_2, 2);
  archivo.print(",");

  archivo.print(velocidadPromedio_kmh, 2);
  archivo.print(",");

  archivo.print(voltaje, 2);
  archivo.print(",");

  archivo.print(corriente, 3);
  archivo.print(",");

  archivo.print(temperatura_a, 2);
  archivo.print(",");

  archivo.print(temperatura_b, 2);
  archivo.print(",");

  archivo.print(lat, 6);
  archivo.print(",");

  archivo.print(lng, 6);
  archivo.print(",");

  archivo.println(gpsSpeed, 2);

  archivo.flush();
  archivo.close();

  Serial.println("SD: dato guardado");
}

// ================= LORA =================

void enviarLoRa() {
  char resultado[58];

  snprintf(resultado, sizeof(resultado),
           "s,%.1f,%.1f,%.0f,%.0f,%.1f,%.4f,%.4f,f",
           voltaje,
           corriente,
           temperatura_a,
           temperatura_b,
           velocidadPromedio_kmh,
           lat,
           lng);

  Serial.print("Bytes mensaje: ");
  Serial.println(strlen(resultado));

  ResponseStatus rs =
      e32ttl.sendFixedMessage(0, 3, 0x04, resultado);

  Serial.println(rs.getResponseDescription());
  Serial.println(resultado);
}

// ================= TAREAS RTOS =================

void tareaSensores(void *parameter) {
  for (;;) {
    leerGPS();
    calcularVelocidad();
    leerCorrienteVoltaje();
    leerTemperaturas();
    guardarEnSD();
    enviarLoRa();

    Serial.printf("V: %.2f V | I: %.3f A | T1: %.2f | T2: %.2f | Vel: %.2f km/h | GPS: %.6f, %.6f\n",
                  voltaje,
                  corriente,
                  temperatura_a,
                  temperatura_b,
                  velocidadPromedio_kmh,
                  lat,
                  lng);

    vTaskDelay(intervaloMedicion_ms / portTICK_PERIOD_MS);
  }
}

void tareaSerial(void *parameter) {
  for (;;) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == 'z' || c == 'Z') {
        Serial.println("Recalibrando cero ACS712...");
        I_ZERO_mV = calibrarCero_mV();
        Serial.print("Nuevo I_ZERO_mV = ");
        Serial.println(I_ZERO_mV, 1);
      }
    }

    leerGPS();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nInicializando sistema ESP32 + RTOS + LoRa + Sensores...");

  analogReadResolution(12);
  analogSetPinAttenuation(SensorPin_V, ADC_11db);
  analogSetPinAttenuation(SensorPin_C, ADC_11db);
  analogSetPinAttenuation(sensorPin_T1, ADC_11db);
  analogSetPinAttenuation(sensorPin_T2, ADC_11db);

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);

  configurarLoRaNormal();

  Serial2.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  e32ttl.begin();

  Serial.println("LoRa E32 iniciado en modo normal.");

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS iniciado.");

  pinMode(SENSOR_HALL_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_HALL_PIN_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_1), pulsoDetectado1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL_PIN_2), pulsoDetectado2, FALLING);

  Serial.println("Calibrando cero de corriente. No debe haber consumo en el ACS712...");
  I_ZERO_mV = calibrarCero_mV();
  Serial.print("I_ZERO_mV = ");
  Serial.println(I_ZERO_mV, 1);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("No se pudo inicializar la SD.");
  } else {
    Serial.println("SD inicializada.");

    if (!SD.exists("/mediciones.csv")) {
      File archivo = SD.open("/mediciones.csv", FILE_WRITE);
      if (archivo) {
        archivo.println("Tiempo_s,Velocidad_S1_kmh,Velocidad_S2_kmh,Velocidad_Prom_kmh,Voltaje_V,Corriente_A,Temp1_C,Temp2_C,Lat,Lng,GPS_Speed_kmh");
        archivo.close();
      }
    }
  }

  xTaskCreatePinnedToCore(
    tareaSensores,
    "Tarea_Sensores_LoRa",
    6000,
    NULL,
    2,
    &TareaSensores,
    1
  );

  xTaskCreatePinnedToCore(
    tareaSerial,
    "Tarea_Serial_GPS",
    4000,
    NULL,
    1,
    &TareaSerial,
    0
  );
}

// ================= LOOP =================

void loop() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
}