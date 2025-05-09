#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include "rgb_lcd.h"
// #define SCAN_I2C_ID

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

// Pins des LED
#define LED_VERTE 0
#define LED_ROUGE 2
int8_t test_led = 0;
// Variables globales
long double val_tick_gauche = 0;
long double val_tick_droite = 0;

// Mutex pour protéger le bus I²C
SemaphoreHandle_t i2cMutex;

#define time_wait_init 10
bool FlagCalcul = 0;
float Te = 5;
float Tau = 250;
void controle(void *parameters)
{
  // Demander a M.CLAMAINS toute information complémentaire sur la ligne suivante
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1)
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

    FlagCalcul = true;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));

    // Demander a M.CLAMAINS toute information complémentaire sur la ligne suivante
  }
}

void setup()
{
  Serial.begin(115200);
#ifdef SCAN_I2C_ID
  Wire.begin();
  Serial.println("Scanning I2C bus...");
  for (byte i = 1; i < 127; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("I2C device found at address 0x");
      Serial.println(i, HEX);
    }
  }
  Serial.println("Scan complete.");
#endif
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
  lcd.clear();
  lcd.print("LCD OK");
  Serial.println("Écran LCD initialisé.");
  delay(time_wait_init);
  // Initialisation du MPU6050
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
  {
    lcd.clear();
    lcd.print("Init MPU6050...");
  }

  if (!mpu.begin())
  {
    Serial.println("Erreur : initialisation du capteur MPU6050 échouée !");
    lcd.clear();
    lcd.print("Erreur INIT...");

    while (1)
      yield();
  }
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  Serial.println("Capteur MPU6050 détecté !");
  xSemaphoreGive(i2cMutex);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 OK");
  lcd.setCursor(0, 1);
  lcd.print("MPU6050 détecté !");
  delay(time_wait_init);
  // Initialisation des encodeurs

  lcd.clear();
  lcd.print("Init Encodeurs...");

  ESP32Encoder::useInternalWeakPullResistors = UP; // Utilise les résistances internes
  encoderdroite.attachHalfQuad(PIN_A_ENCODEUR_DROITE, PIN_B_ENCODEUR_DROITE);
  encodergauche.attachHalfQuad(PIN_A_ENCODEUR_GAUCHE, PIN_B_ENCODEUR_GAUCHE);
  encoderdroite.clearCount();
  encodergauche.clearCount();

  Serial.println("Encodeurs initialisés.");
  lcd.clear();
  lcd.print("Encodeurs OK");
  delay(time_wait_init);

  Serial.println("LED init");
  lcd.clear();
  lcd.print("LED init");

  pinMode(LED_VERTE, OUTPUT);
  pinMode(LED_ROUGE, OUTPUT);

  while (test_led < (10))
  {
    digitalWrite(LED_VERTE, true);
    digitalWrite(LED_ROUGE, true);
    Serial.println("Allume");
    delay(time_wait_init);
    digitalWrite(LED_VERTE, false);
    digitalWrite(LED_ROUGE, false);
    Serial.println("eteint");
    delay(time_wait_init);
    test_led++;
  }

  lcd.clear();
  lcd.print("LED OK");
  delay(time_wait_init);

  // Finalisation
  lcd.clear();
  lcd.setRGB(0, 127, 0); // Couleur verte pour signaler la fin
  lcd.print("Init Terminee!");
  delay(time_wait_init);
  xSemaphoreGive(i2cMutex);

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      1,          // tres haut niveau de priorite
      NULL        // descripteur
  );

  Serial.println("Initialisation terminée.");
}
void loop()
{
  if (FlagCalcul)
  {
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
    FlagCalcul = false;
  }
}
