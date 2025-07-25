#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h> // Usamos BMP280, no BME280
#include <Adafruit_Sensor.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050_light.h>
#include <math.h>

//BMP280
#define SEALEVELPRESSURE_HPA (1024) // Presión estándar o la de tu zona
Adafruit_BMP280 bmp; // I2C
const int N = 10;             // Cantidad de muestras para promedio
float altitudes[N];           // Array para guardar muestras
int indexAlt = 0;
bool llenadoCompleto = false;

//MPU6050
float inclinacionX = 0.0;
float inclinacionY = 0.0;
float inclinacionZ = 0.0;
float aceleracionX =  0.0;
float aceleracionY =  0.0;
float aceleracionZ =  0.0;
float aceleracionTotal =  0.0;

//DS18B20
const int pinDatosDQ = 12;
float tempAgua = 0.0;
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

//MPU6050
MPU6050 mpu(Wire);
const float gravedad = 9.80665;
//Pagina web
const char* ssid = "red wifi";
const char* password = "contraseña wifi";

WebServer server(80);

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
html += "<style>body{text-align:center;font-family:sans-serif;}table{margin:auto;}th,td{padding:10px;}</style></head><body>";
html += "<h1>Nautilus VI</h1>";

html += "<h2>Lecturas del BMP280</h2><table border='1'>";
html += "<tr><th>Temperatura interna del submarino(grados C)</th><td>" + String(bmp.readTemperature()) + "</td></tr>";
html += "<tr><th>Presión (hPa)</th><td>" + String(bmp.readPressure() / 100.0F) + "</td></tr>";
html += "<tr><th>Altitud estimada (m)</th><td>" + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "</td></tr>";
html += "</table>";

html += "<h2>Lecturas del DS18B20</h2><table border='1'>";
html += "<tr><th>Temperatura del agua(grados C)</th><td>" + String(tempAgua) + "</td></tr>";
html += "</table>";

html += "<h2>Lecturas del MPU6050</h2><table border='1'>";
html += "<tr><th>inclinacion en el eje X</th><td>" + String(inclinacionX) + "</td></tr>";
html += "<tr><th>inclinacion en el eje Y</th><td>" + String(inclinacionY) + "</td></tr>";
html += "<tr><th>inclinacion en el eje Z</th><td>" + String(inclinacionZ) + "</td></tr>";
html += "<tr><th>Aceleracion en el eje X</th><td>" + String(aceleracionX) + "</td></tr>";
html += "<tr><th>Aceleracion en el eje Y</th><td>" + String(aceleracionY) + "</td></tr>";
html += "<tr><th>Aceleracion en el eje Z</th><td>" + String(aceleracionZ) + "</td></tr>";
html += "<tr><th>Aceleracion total</th><td>" + String(aceleracionTotal) + "</td></tr>";
html += "</table>";

html += "</body></html>";

  
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  sensorDS18B20.begin(); 
  delay(1000);
  if (!bmp.begin(0x76)) {
    Serial.println("No se encontró un sensor BMP280 válido. Verifica el cableado.");
    while (1);
  }

  //MPU6050
    Wire.begin(18, 19);  // tus pines SDA y SCL personalizados

  delay(1000);  // Esperar un poco para que el sensor arranque

  Serial.println("Inicializando MPU6050...");
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("Error de conexión: ");
    Serial.println(status);
    while (1);
  }

  Serial.println("Calibrando...");
  mpu.calcOffsets();  // Calibra automáticamente
  Serial.println("Listo!");


  Serial.println("Conectando al WiFi...");
  WiFi.begin(ssid, password);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    intentos++;
    if (intentos > 40) { // 20 segundos de espera
      Serial.println("\nNo se pudo conectar al WiFi. Verifica SSID y contraseña o señal.");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado. Dirección IP:");
    Serial.println(WiFi.localIP());
  }
  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  mpu.update();
  float inclinacionX = mpu.getAccX() * gravedad;
  float inclinacionY = mpu.getAccY() * gravedad;
  float inclinacionZ = mpu.getAccZ() * gravedad;
  float aceleracionX =  mpu.getAccX() * gravedad;
  float aceleracionY =  mpu.getAccX() * gravedad;
  float aceleracionZ =  mpu.getAccX() * gravedad;
  float aceleracionTotal = sqrt(aceleracionX * aceleracionX + aceleracionY * aceleracionY + aceleracionZ * aceleracionZ);
  server.handleClient();
  sensorDS18B20.requestTemperatures();
  tempAgua = sensorDS18B20.getTempCByIndex(0);
}
