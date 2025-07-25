#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  
  // Configura los pines SDA y SCL según tu conexión
  Wire.begin(18, 19);  // Por ejemplo, SDA = GPIO18, SCL = GPIO19
  delay(1000);

  Serial.println("Escaneando bus I2C...");

  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();

      nDevices++;
    }
  }

  if (nDevices == 0)
    Serial.println("No se encontraron dispositivos I2C.");
  else
    Serial.println("Escaneo finalizado.");
}

void loop() {
  // Nada aquí
}
