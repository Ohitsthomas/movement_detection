
#include <Wire.h>
#include <VL53L0X.h>

#define XSHUT_1 2
#define XSHUT_2 3

VL53L0X sensor1;
VL53L0X sensor2;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  // Beide Sensoren ausschalten
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(100);

  // Sensor 1 einschalten und initialisieren
  digitalWrite(XSHUT_1, HIGH);
  delay(100);
  if (!sensor1.init()) {
    Serial.println("Sensor 1 nicht gefunden!");
    while (1);
  }
  sensor1.setAddress(0x30);

  // Sensor 2 einschalten und initialisieren
  digitalWrite(XSHUT_2, HIGH);
  delay(100);
  if (!sensor2.init()) {
    Serial.println("Sensor 2 nicht gefunden!");
    while (1);
  }
  sensor2.setAddress(0x31);

  sensor1.setTimeout(500);
  sensor2.setTimeout(500);
}

void loop() {
  uint16_t dist1 = sensor1.readRangeSingleMillimeters();
  uint16_t dist2 = sensor2.readRangeSingleMillimeters();

  if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred()) {
    Serial.println("timeout");
  } else {
    // Ausgabe als "dist1,dist2\n"
    Serial.print(dist1);
    Serial.print(",");
    Serial.println(dist2);
  }

  delay(100);  // 10 Messungen pro Sekunde
}