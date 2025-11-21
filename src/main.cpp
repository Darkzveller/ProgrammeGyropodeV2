#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include "rgb_lcd.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

// #define SCAN_I2C_ID

// Déclaration des périphériques
Adafruit_MPU6050 mpu;
rgb_lcd lcd;

ESP32Encoder encodergauche;
ESP32Encoder encoderdroite;

// Déclaration des événements du MPU6050
sensors_event_t a, g, temp;
double angle = 0;
double z = 0;
double y = 0;
double x = 0;

// Pins des encodeurs
#define PIN_A_ENCODEUR_GAUCHE 33
#define PIN_B_ENCODEUR_GAUCHE 32
#define PIN_A_ENCODEUR_DROITE 26
#define PIN_B_ENCODEUR_DROITE 25
// Variables globales encoder
long double val_tick_gauche = 0;
long double val_tick_droite = 0;

// Pin des moteur
int frequence = 19000;
int resolution = 12;
#define PIN_A_MOTEUR_GAUCHE 17
#define PIN_B_MOTEUR_GAUCHE 16
#define PIN_A_MOTEUR_DROITE 19
#define PIN_B_MOTEUR_DROITE 18

#define CANAL_MOTEUR_DROIT_1 1
#define CANAL_MOTEUR_DROIT_2 2
#define CANAL_MOTEUR_GAUCHE_3 3
#define CANAL_MOTEUR_GAUCHE_4 4
double frottement_moteur = 0.20;
double frottement_moteur_gauche = 0.0;
double frottement_moteur_droite = 0.15;

void init_moteur(bool activate);
void moteur_gauche(int pwm);
void moteur_droite(int pwm);

// Pins des LED
#define LED_VERTE 0
#define LED_ROUGE 2
int8_t test_led = 0;
// Pin mesure batterie
#define PIN_MESURE_TENSION 36
#define PIN_INTERRUPTEUR_BATTERIE 27
// esp_adc_cal_characteristics_t adc_chars; // allocation statique
double mesure_bat(int pin, int nb_lectures = 50);

// Mutex pour protéger le bus I²C
SemaphoreHandle_t i2cMutex;

#define time_wait_init_ms 100
double A, B;
bool FlagCalcul = 0;
float Te = 5;
float Tau = 250;
double thetaG = 0;
double thetaGF = 0;
double thetaW = 0;
double thetaWF = 0;
double theta = 0;
double erreur_pos_ang = 0;
double cons = 0;
double cons_equilibre = 0;
double commande = 0;
double commande_gauche = 0;
double commande_droite = 0;

double Kp_position = 19.6;
double Kd_position = -0.83;
double Ki_postion = 0;
double somme_integral_postion = 0;
double integral_limite_position = 0.25;
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
      static bool one_pass = true;
      if (one_pass)
      {
        for (int i = 0; i <= 1; i++)
        {
          delay(2000);
        }
        digitalWrite(PIN_INTERRUPTEUR_BATTERIE, HIGH);
        one_pass = false;
      }
      mpu.getEvent(&a, &g, &temp);
      xSemaphoreGive(i2cMutex); // Libérer le mutex
      z = a.acceleration.z;
      y = a.acceleration.y;
      x = a.acceleration.x;
      angle = degrees(atan2(y, z));

      // Calcul de théta a l'aide de l'accélération mesurer
      thetaG = atan2(y, z); // Permet de calculer l'angle Théta G avec un angle dans la valeur est entier relatif
      // Calcul du Théta filtrer
      thetaGF = A * thetaG + B * thetaGF;

      // Calcul de théta a l'aide de la valeur mesurer par le gyroscope
      thetaW = g.gyro.x * Tau * 1e-3;
      // Calcul du thétaW filtrer
      thetaWF = A * thetaW + B * thetaWF;

      // Calcule de la somme permettant d'avoir un passe bande filtrer
      theta = thetaGF + thetaWF;
      erreur_pos_ang = (cons + cons_equilibre) - theta;
      float proportionnel = erreur_pos_ang * Kp_position;
      float derivee = Kd_position * 1 * g.gyro.x;

      somme_integral_postion += erreur_pos_ang * Te;
      if (somme_integral_postion > integral_limite_position)
      {
        somme_integral_postion = integral_limite_position;
      }
      else if (somme_integral_postion < -integral_limite_position)
      {
        somme_integral_postion = -integral_limite_position;
      }

      double integral = Ki_postion * somme_integral_postion;

      commande = proportionnel + derivee + integral;

      if (commande < 0)
      {
        // commande = commande - frottement_moteur;
        commande_gauche = commande - frottement_moteur_gauche;
        commande_droite = commande - frottement_moteur_droite;
      }
      else if (commande > 0)
      {
        // commande = commande + frottement_moteur;
        commande_gauche = commande + frottement_moteur_gauche;
        commande_droite = commande + frottement_moteur_droite;
      }
      commande_gauche = constrain(commande_gauche, -0.45, +0.45);
      commande_droite = constrain(commande_droite, -0.45, +0.45);
      commande_gauche = commande_gauche + 0.5;
      commande_droite = commande_droite + 0.5;
      float pwm_gauche = commande_gauche * ((1 << resolution) - 1);
      float pwm_droite = commande_droite * ((1 << resolution) - 1);

      moteur_gauche((int)pwm_gauche);
      moteur_droite((int)pwm_droite);

      // Serial.print(x);
      // Serial.printf(" ");
      // Serial.print(y);
      // Serial.printf(" ");
      // Serial.print(z);
      // Serial.printf(" ");
      // Serial.print(angle);
      // Serial.printf(" ");
      // Serial.print((double)val_tick_gauche);
      // Serial.printf(" ");
      // Serial.print((double)val_tick_droite);
      // Serial.printf(" ");
      // Serial.print((double)mesure_bat(PIN_MESURE_TENSION, 50));
    }
    // moteur_gauche((int)4095);
    // moteur_droite((int)4095);

    // // Lecture des ticks des encodeurs
    // val_tick_gauche = encodergauche.getCount();
    // val_tick_droite = encoderdroite.getCount();

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
    delay(time_wait_init_ms);
    xSemaphoreGive(i2cMutex); // Libérer le mutex après l'initialisation
  }
  lcd.clear();
  lcd.print("LCD OK");
  Serial.println("Écran LCD initialisé.");
  delay(time_wait_init_ms);
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
  delay(time_wait_init_ms);
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
  delay(time_wait_init_ms);

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
    delay(time_wait_init_ms);
    digitalWrite(LED_VERTE, false);
    digitalWrite(LED_ROUGE, false);
    Serial.println("eteint");
    delay(time_wait_init_ms);
    test_led++;
  }

  lcd.clear();
  lcd.print("LED OK");
  delay(time_wait_init_ms);

  lcd.clear();
  lcd.print("Init Moteurs...");

  init_moteur(true);
  Serial.println("Moteurs initialisés.");
  lcd.clear();
  lcd.print("Moteur OK");
  delay(time_wait_init_ms);

  lcd.clear();
  lcd.print("Init Mesure tension...");
  pinMode(PIN_MESURE_TENSION, INPUT);
  analogReadResolution(12);
  pinMode(PIN_INTERRUPTEUR_BATTERIE, OUTPUT);
  digitalWrite(PIN_INTERRUPTEUR_BATTERIE, LOW);

  Serial.println("Mesure tension initialisés.");
  lcd.clear();
  lcd.print("Mesure bat OK");
  delay(time_wait_init_ms);

  // Finalisation
  lcd.clear();
  lcd.setRGB(0, 127, 0); // Couleur verte pour signaler la fin
  lcd.print("Init Terminee!");
  delay(time_wait_init_ms);
  xSemaphoreGive(i2cMutex);

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      1,          // tres haut niveau de priorite
      NULL        // descripteur
  );
  // ! Calcul des coefficents du filtre sans développement des expressions de la fonction de transfert
  A = 1 / (1 + Tau / Te);
  B = A * (Tau / Te);

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
      Serial.print(erreur_pos_ang);
      Serial.printf(" ");
      Serial.print(commande);
      Serial.printf(" ");
      Serial.print(commande_droite);
      Serial.printf(" ");
      Serial.print(commande_gauche);
      Serial.println();
    }
    FlagCalcul = false;
  }
}

void init_moteur(bool activate)
{
  if (activate)
  {
    // Config Moteur
    ledcSetup(CANAL_MOTEUR_DROIT_1, frequence, resolution);
    ledcSetup(CANAL_MOTEUR_DROIT_2, frequence, resolution); // <--- MANQUANT
    // Connexion entre les pins et les canaux
    ledcAttachPin(PIN_A_MOTEUR_DROITE, CANAL_MOTEUR_DROIT_1);
    ledcAttachPin(PIN_B_MOTEUR_DROITE, CANAL_MOTEUR_DROIT_2);

    ledcSetup(CANAL_MOTEUR_GAUCHE_3, frequence, resolution);
    ledcSetup(CANAL_MOTEUR_GAUCHE_4, frequence, resolution); // <--- MANQUANT

    // Connexion entre les pins et les canaux
    ledcAttachPin(PIN_A_MOTEUR_GAUCHE, CANAL_MOTEUR_GAUCHE_3);
    ledcAttachPin(PIN_B_MOTEUR_GAUCHE, CANAL_MOTEUR_GAUCHE_4);
    Serial.printf("canal 1 = %d ", ledcRead(CANAL_MOTEUR_DROIT_1));
    Serial.printf("canal 2 = %d", ledcRead(CANAL_MOTEUR_DROIT_2));
    Serial.printf("canal 3 = %d ", ledcRead(CANAL_MOTEUR_GAUCHE_3));
    Serial.printf("canal 4 = %d \n", ledcRead(CANAL_MOTEUR_GAUCHE_4));
  }
}

void moteur_droite(int pwm)
{
  pwm = constrain(pwm, 0, ((1 << resolution) - 1));
  int reverse_pwm;
  reverse_pwm = ((1 << resolution) - 1) - pwm;
  ledcWrite(CANAL_MOTEUR_DROIT_1, reverse_pwm);
  ledcWrite(CANAL_MOTEUR_DROIT_2, pwm);
  // Serial.printf(" pwmd %d ", pwm);
  // Serial.printf(" ");
  // Serial.print(reverse_pwm);
  // Serial.printf(" ");
}
void moteur_gauche(int pwm)
{
  pwm = constrain(pwm, 0, ((1 << resolution) - 1));
  int reverse_pwm;
  reverse_pwm = ((1 << resolution) - 1) - pwm;
  ledcWrite(CANAL_MOTEUR_GAUCHE_3, pwm);
  ledcWrite(CANAL_MOTEUR_GAUCHE_4, reverse_pwm);
  // Serial.printf("pwmg %d ", pwm);
  // Serial.printf(" ");
  // Serial.print(reverse_pwm);
  // Serial.printf(" ");
}

double mesure_bat(int pin, int nb_lectures)
{
  int adcValue = analogRead(pin); // 0 à 4095
  long somme = 0;                 // utiliser long pour éviter overflow
  for (int i = 0; i < nb_lectures; i++)
  {
    somme += analogRead(pin); // additionner toutes les lectures
    delayMicroseconds(50);    // petit délai pour stabilité
  }

  int adcMoyenne = somme / (int)nb_lectures;
  double Vout = (adcMoyenne * 3.3) / 4095.0;

  double Vin = Vout * ((15000.0 + 5600.0) / 5600.0) * 1.05;
  return Vin;
}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
    }
    if (commande == "C")
    {
      Kp_position = valeur.toDouble();
    }
    if (commande == "Kd")
    {
      Kd_position = valeur.toDouble();
    }
    if (commande == "Ki")
    {
      Ki_postion = valeur.toDouble();
    }

    if (commande == "Fr_g")
    {
      frottement_moteur_gauche = valeur.toDouble();
    }

    if (commande == "Fr_d")
    {
      frottement_moteur_gauche = valeur.toDouble();
    }

    // if (commande == "consE")
    // {
    //   consE = valeur.toFloat();
    //   // Serial.println(consE);
    // }
    // if (commande == "Kw")
    // {
    //   Kw = valeur.toFloat();
    // }
    // if (commande == "Kwd")
    // {
    //   Kwd = valeur.toFloat();
    // }

    // if (commande == "consVit")
    // {
    //   CONSv = valeur.toFloat();
    // }
    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}
