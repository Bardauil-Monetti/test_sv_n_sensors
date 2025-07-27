#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h> // BMP280
#include <Adafruit_Sensor.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h> // DS18B20
#include <MPU6050_light.h>
#include <math.h>

#define SEALEVELPRESSURE_HPA (1024) // Presión estándar

// BMP280
Adafruit_BMP280 bmp; // I2C

// MPU6050
MPU6050 mpu(Wire);
const float gravedad = 9.80665;
float inclinacionX = 0.0;
float inclinacionY = 0.0;
float inclinacionZ = 0.0;
float aceleracionX = 0.0;
float aceleracionY = 0.0;
float aceleracionZ = 0.0;
float aceleracionTotal = 0.0;

// DS18B20
const int pinDatosDQ = 14;
float tempAgua = 0.0;
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

// WiFi
const char* red = "Telecentro-ece8";
const char* contra = "DPYD4HHFQ37C";

WebServer server(80);

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<style>body{text-align:center;font-family:sans-serif;}table{margin:auto;}th,td{padding:10px;}</style></head><body>";
  html += "<h1>Nautilus VI</h1>";

  html += "<h2>Lecturas del BMP280</h2><table border='1'>";
  html += "<tr><th>Temperatura interna del submarino (grados C)</th><td>" + String(bmp.readTemperature()) + "</td></tr>";
  html += "<tr><th>Presion (hPa)</th><td>" + String(bmp.readPressure() / 100.0F) + "</td></tr>";
  html += "<tr><th>Altitud estimada (m)</th><td>" + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "</td></tr>";
  html += "</table>";

  html += "<h2>Lecturas del DS18B20</h2><table border='1'>";
  html += "<tr><th>Temperatura del agua (grados C)</th><td>" + String(tempAgua) + "</td></tr>";
  html += "</table>";

  html += "<h2>Lecturas del MPU6050</h2><table border='1'>";
  html += "<tr><th>Inclinacion en eje X</th><td>" + String(inclinacionX) + "</td></tr>";
  html += "<tr><th>Inclinacion en eje Y</th><td>" + String(inclinacionY) + "</td></tr>";
  html += "<tr><th>Inclinacion en eje Z</th><td>" + String(inclinacionZ) + "</td></tr>";
  html += "<tr><th>Aceleracion X</th><td>" + String(aceleracionX) + "</td></tr>";
  html += "<tr><th>Aceleracion Y</th><td>" + String(aceleracionY) + "</td></tr>";
  html += "<tr><th>Aceleracion Z</th><td>" + String(aceleracionZ) + "</td></tr>";
  html += "<tr><th>Aceleracion Total</th><td>" + String(aceleracionTotal) + "</td></tr>";
  html += "</table></body></html>";

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  sensorDS18B20.begin();

  Wire.begin(21, 22); // Pines I2C comunes para sensores MPU6050 y BMP280

  Serial.println("Iniciando BMP280...");
  if (!bmp.begin(0x76)) {
    Serial.println("No se encontro el BMP280. Verifica el cableado.");
    while (1);
  }

  delay(1000);

  Serial.println("Inicializando MPU6050...");
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("Error de conexión: ");
    Serial.println(status);
    while (1);
  }
  Serial.println("Calibrando...");
  mpu.calcOffsets();
  Serial.println("Listo!");

  Serial.println("Conectando al WiFi...");
  WiFi.begin(red, contra);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++intentos > 40) {
      Serial.println("\n No se pudo conectar al WiFi.");
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
  }

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  mpu.update();

  inclinacionX = mpu.getAccX();
  inclinacionY = mpu.getAccY();
  inclinacionZ = mpu.getAccZ();

  aceleracionX = mpu.getAccX() * gravedad;
  aceleracionY = mpu.getAccY() * gravedad;
  aceleracionZ = mpu.getAccZ() * gravedad;

  aceleracionTotal = sqrt(aceleracionX * aceleracionX + aceleracionY * aceleracionY + aceleracionZ * aceleracionZ);

  server.handleClient();

  sensorDS18B20.requestTemperatures();
  tempAgua = sensorDS18B20.getTempCByIndex(0);
}
