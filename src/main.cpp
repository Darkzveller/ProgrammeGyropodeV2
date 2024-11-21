/*#include <Arduino.h>
// #include <Wire.h>
#include <ESP32Encoder.h>

// Bibliotheque MPU 6050
#include <Adafruit_MPU6050.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_Sensor.h>
#include "rgb_lcd.h"

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

#define PIN_A_ENCODEUR_GAUCHE 33
#define PIN_B_ENCODEUR_GAUCHE 32

#define PIN_A_ENCODEUR_DROITE 26
#define PIN_B_ENCODEUR_DROITE 25
long double val_tick_gauche = 0;
long double val_tick_droite = 0;

rgb_lcd lcd;
ESP32Encoder encodergauche;
ESP32Encoder encoderdroite;

void setup()
{
  Serial.begin(115200);
  if (!mpu.begin())
  {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");
  // On augmente la précision de la mesure effectuer avec le gyroscope
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  lcd.begin(16, 2);

  // Print a message to the LCD.
  lcd.print("hello, world!");

  encoderdroite.attachHalfQuad(PIN_A_ENCODEUR_DROITE, PIN_B_ENCODEUR_DROITE);
  encodergauche.attachHalfQuad(PIN_A_ENCODEUR_GAUCHE, PIN_B_ENCODEUR_GAUCHE);

  encoderdroite.clearCount();
  encodergauche.clearCount();
  delay(1000);
}

void loop()
{
  // Acquissition des toutes les mesures
  mpu.getEvent(&a, &g, &temp);
  float z = a.acceleration.z;
  float y = a.acceleration.y;
  float x = a.acceleration.x;
  Serial.print(x);
  Serial.printf(" ");
  Serial.print(y);
  Serial.printf(" ");
  Serial.print(z);
  float angle = degrees(atan2(y, z));
  Serial.printf(" ");
  Serial.print(angle);

  Serial.println();

  val_tick_gauche = encodergauche.getCount();
  val_tick_droite = encoderdroite.getCount();
  lcd.clear();
  lcd.setRGB(127, 0, 127);
  lcd.setCursor(0, 0);
  lcd.printf("G %d", val_tick_gauche);
  lcd.setCursor(0, 1);
  lcd.printf("D %d", val_tick_droite);

  // Serial.print((double)val_tick_gauche);
  // Serial.printf(" ");
  // Serial.println((double)val_tick_droite);
  delay(50);
}
*/
/*
#include <Arduino.h>
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Scanning I2C bus...");
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(i, HEX);
    }
  }
  Serial.println("Scan complete.");
}

void loop() {}
*/
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include "rgb_lcd.h"

// Déclaration des périphériques
Adafruit_MPU6050 mpu;
rgb_lcd lcd;

ESP32Encoder encodergauche;
ESP32Encoder encoderdroite;

// Déclaration des événements du MPU6050
sensors_event_t a, g, temp;

// Pins des encodeurs
#define PIN_A_ENCODEUR_GAUCHE 33
#define PIN_B_ENCODEUR_GAUCHE 32
#define PIN_A_ENCODEUR_DROITE 26
#define PIN_B_ENCODEUR_DROITE 25

// Variables globales
long double val_tick_gauche = 0;
long double val_tick_droite = 0;

// Mutex pour protéger le bus I²C
SemaphoreHandle_t i2cMutex;

float time_wait_init = 2500;
void setup()
{
  Serial.begin(115200);

  // Initialisation du mutex
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL)
  {
    Serial.println("Erreur : création du mutex I2C échouée !");
    while (1)
      ;
  }

  // Initialisation de l'écran LCD
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
  {
    lcd.begin(16, 2);
    lcd.setRGB(127, 127, 0); // Couleur jaune pour l'initialisation
    lcd.print("Init LCD...");
    delay(time_wait_init);
    xSemaphoreGive(i2cMutex); // Libérer le mutex après l'initialisation
  }
  Serial.println("Écran LCD initialisé.");

  // Initialisation du MPU6050
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
  {
    lcd.clear();
    lcd.print("Init MPU6050...");
  }

  if (!mpu.begin())
  {
    Serial.println("Erreur : initialisation du capteur MPU6050 échouée !");
    while (1)
      yield();
  }
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  Serial.println("Capteur MPU6050 détecté !");
  xSemaphoreGive(i2cMutex);

  lcd.clear();
  lcd.print("MPU6050 OK");
  delay(time_wait_init);
  // Initialisation des encodeurs

  lcd.clear();
  lcd.print("Init Encodeurs...");
  delay(time_wait_init);

  ESP32Encoder::useInternalWeakPullResistors = UP; // Utilise les résistances internes
  encoderdroite.attachHalfQuad(PIN_A_ENCODEUR_DROITE, PIN_B_ENCODEUR_DROITE);
  encodergauche.attachHalfQuad(PIN_A_ENCODEUR_GAUCHE, PIN_B_ENCODEUR_GAUCHE);
  encoderdroite.clearCount();
  encodergauche.clearCount();

  Serial.println("Encodeurs initialisés.");
  lcd.clear();
  lcd.print("Encodeurs OK");
  delay(time_wait_init);

  // Finalisation

  lcd.clear();
  lcd.setRGB(0, 127, 0); // Couleur verte pour signaler la fin
  lcd.print("Init Terminee!");
  delay(time_wait_init);
  xSemaphoreGive(i2cMutex);

  Serial.println("Initialisation terminée.");
}
void loop()
{
  // Lecture des données MPU6050
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
  { // Prendre le mutex
    mpu.getEvent(&a, &g, &temp);
    xSemaphoreGive(i2cMutex); // Libérer le mutex
    float z = a.acceleration.z;
    float y = a.acceleration.y;
    float x = a.acceleration.x;
    float angle = degrees(atan2(y, z));

    Serial.print(x);
    Serial.printf(" ");
    Serial.print(y);
    Serial.printf(" ");
    Serial.print(z);
    Serial.printf(" ");
    Serial.print(angle);
    Serial.println();
  }

  // Lecture des ticks des encodeurs
  val_tick_gauche = encodergauche.getCount();
  val_tick_droite = encoderdroite.getCount();

  // Mise à jour de l'écran LCD
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
  { // Prendre le mutex
    lcd.clear();
    lcd.setRGB(127, 0, 127);
    lcd.setCursor(0, 0);
    lcd.printf("G %d", (int)val_tick_gauche);
    lcd.setCursor(0, 1);
    lcd.printf("D %d", (int)val_tick_droite);
    xSemaphoreGive(i2cMutex); // Libérer le mutex
  }

  delay(50); // Pause pour éviter des mises à jour trop rapides
}
