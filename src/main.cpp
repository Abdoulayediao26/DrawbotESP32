/* =========================================================================
   DRAWBOT — Firmware ESP32 (BLE) — Version qui MARCHE
   Projet Systèmes Bouclés — ECE Paris
   -------------------------------------------------------------------------
   Cette version reprend INTACT le code de séquences fonctionnel
   (fairAngleDroit, dessinerCercle, dessinerFlecheNord, calibrerMag) du
   projet utilisateur, et remplace juste la couche WiFi/WebServer/WebSocket
   par du Bluetooth Low Energy (BLE).

   Améliorations :
   - Communication via BLE (pas besoin d'être sur le même WiFi)
   - Flag stopRequest dans toutes les boucles while → STOP interrompt VRAIMENT
   - Compatible avec l'interface React drawbot_interface.html
   -------------------------------------------------------------------------
   Bibliothèques requises : AUCUNE bibliothèque externe !
   - BLE est inclus dans le core ESP32 by Espressif
   - LSM6DS3 et LIS3MDL sont lus directement en I2C (pas de lib SparkFun)
   ========================================================================= */

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================
// PINS
// ============================================================
#define LEDU1 25
#define LEDU2 26
#define LED_BUILTIN 2

#define EN_D 23
#define EN_G 4

#define IN_1_D 19
#define IN_2_D 18

#define IN_1_G 17
#define IN_2_G 16
#define PWM_OFFSET_D 20

#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14

#define SDA 21
#define SCL 22

#define ADDR_IMU 0x6B
#define ADDR_MAG 0x1E

// ============================================================
// CONSTANTES MOTEURS
// ============================================================
#define VITESSE 128
#define PWM_AVANCE 100
#define PWM_PIVOT 120
#define PWM_AVANCE_G 100
#define PWM_AVANCE_D 120

// ============================================================
// CONSTANTES CERCLE
// ============================================================
#define ECARTEMENT_ROUES_MM 88.0
#define DISTANCE_STYLO_MM 130.0
#define RAYON_MIN_MM 130.0
#define PWM_CERCLE_BASE 100.0

// ============================================================
// CONSTANTES ANGLE DROIT
// ============================================================
#define CRANS_PAR_TOUR 360
#define DIAMETRE_ROUE_MM 65.0
#define ANGLE_PAS_DEG 1
#define LONGUEUR_PREMIER_TRAIT_MM 300.0
#define ANGLE_TOTAL_DEG 90.0
#define TOLERANCE_GYRO 0.7
#define CRANS_AVANCE 5

// ============================================================
// CALCULS AUTOMATIQUES
// ============================================================
#define CIRCONF_MM (PI * DIAMETRE_ROUE_MM)
#define CRANS_PAR_MM (CRANS_PAR_TOUR / CIRCONF_MM)

// ============================================================
// MAGNÉTOMÈTRE LIS3MDL
// ============================================================
#define LIS3MDL_ADDR     0x1E
#define LIS3MDL_CTRL1    0x20
#define LIS3MDL_CTRL3    0x22
#define LIS3MDL_OUT_X_L  0x28

float magOffsetX = 0;
float magOffsetY = 0;
float anglePivot = 0;
void lireMag(float &mx, float &my);

// ============================================================
// LSM6DS3 (IMU) — lecture directe I2C, pas de lib externe
// ============================================================
#define LSM6DS3_ADDR     0x6B
#define LSM6DS3_WHO_AM_I 0x0F
#define LSM6DS3_CTRL1_XL 0x10   // config accéléromètre
#define LSM6DS3_CTRL2_G  0x11   // config gyroscope
#define LSM6DS3_CTRL3_C  0x12   // config commune
#define LSM6DS3_OUTZ_L_G 0x26   // gyro Z low byte
#define LSM6DS3_OUTZ_H_G 0x27   // gyro Z high byte

// Init IMU : gyro à 416 Hz, plage ±245 dps
void initIMU() {
    // Vérifie WHO_AM_I (doit valoir 0x69 ou 0x6A selon la révision)
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(LSM6DS3_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM6DS3_ADDR, 1);
    uint8_t who = Wire.read();
    Serial.printf("[IMU] WHO_AM_I = 0x%02X\n", who);

    // CTRL1_XL = 0x60 : XL 416 Hz, ±2g
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(LSM6DS3_CTRL1_XL); Wire.write(0x60);
    Wire.endTransmission();

    // CTRL2_G = 0x60 : Gyro 416 Hz, ±245 dps
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(LSM6DS3_CTRL2_G); Wire.write(0x60);
    Wire.endTransmission();

    // CTRL3_C = 0x44 : BDU (Block Data Update) + IF_INC (auto-incrément)
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(LSM6DS3_CTRL3_C); Wire.write(0x44);
    Wire.endTransmission();
}

// Lit le gyro Z en deg/s (équivalent de l'ancien readFloatGyroZ())
float readFloatGyroZ() {
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(LSM6DS3_OUTZ_L_G);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM6DS3_ADDR, 2);
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    int16_t raw = (int16_t)((hi << 8) | lo);
    // Sensibilité ±245 dps : 8.75 mdps/LSB = 0.00875 dps/LSB
    return raw * 0.00875f;
}

// ============================================================
// VARIABLES GLOBALES
// ============================================================
float angleZ = 0;
float angleAvance = 0;
float ly, l2, r2;

volatile long countG = 0;
volatile long countD = 0;
volatile bool stopRequest = false;        // <-- NOUVEAU : interrompt les boucles
volatile bool commandePending = false;    // <-- NOUVEAU : commande BLE à exécuter
String commandePendingStr = "";

bool fonctionEnCours = false;
int erreurprecedente = 0;
int erreurIntegrale = 0;
int erreurDerivee = 0;
float kp = 5;
float ki = 0;
float kd = 0.5;

float pidAvance_erreurPrec = 0;
float pidAvance_integral   = 0;

bool enMarche = false;
float anglePremierPivot = 0;
float rayonCercle = 200.0;

// ============================================================
// BLE — Bluetooth Low Energy (remplace WiFi/WebServer/WebSocket)
// ============================================================
#define DEVICE_NAME       "Drawbot_ECE"
#define SERVICE_UUID      "0000ace0-1234-5678-1234-56789abcdef0"
#define CMD_CHAR_UUID     "0000ace1-1234-5678-1234-56789abcdef0"
#define TLM_CHAR_UUID     "0000ace2-1234-5678-1234-56789abcdef0"

BLEServer*         pServer       = nullptr;
BLECharacteristic* pCmdChar      = nullptr;
BLECharacteristic* pTlmChar      = nullptr;
volatile bool      bleConnected  = false;

class DrawbotServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* s) override {
        bleConnected = true;
        digitalWrite(LEDU2, HIGH);
    }
    void onDisconnect(BLEServer* s) override {
        bleConnected = false;
        digitalWrite(LEDU2, LOW);
        s->startAdvertising();
    }
};

void bleSend(const String &s) {
    Serial.print(s); Serial.print('\n');
    if (!bleConnected || !pTlmChar) return;
    String line = s + "\n";
    const int CHUNK = 180;
    int len = line.length();
    for (int off = 0; off < len; off += CHUNK) {
        int n = min(CHUNK, len - off);
        pTlmChar->setValue((uint8_t*)(line.c_str() + off), n);
        pTlmChar->notify();
        delay(2);
    }
}
void bleSendf(const char* fmt, ...) {
    char buf[200];
    va_list args; va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    bleSend(String(buf));
}

// ============================================================
// INTERRUPTIONS ENCODEURS (identique au code utilisateur)
// ============================================================
void IRAM_ATTR isrG_A() {
    if (digitalRead(ENC_G_CH_B) == HIGH) countG--;
    else countG++;
}
void IRAM_ATTR isrD_A() {
    if (digitalRead(ENC_D_CH_B) == HIGH) countD++;
    else countD--;
}

// ============================================================
// FONCTIONS COMMUNES (identique, avec stopRequest ajouté dans les while)
// ============================================================
long mmEnCrans(float mm) {
    return (long)(mm * CRANS_PAR_MM);
}

void stopper_immediat() {
    enMarche = false;
    fonctionEnCours = false;
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    analogWrite(IN_1_D, 0);
    analogWrite(IN_2_D, 0);
    analogWrite(IN_1_G, 0);
    analogWrite(IN_2_G, 0);
}

void avancerMM(float distanceMM) {
    long cransCible = mmEnCrans(distanceMM);
    countG = 0;
    countD = 0;
    angleZ = 0;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);

    while (countG < cransCible && countD < cransCible && !stopRequest) {
        float erreur = angleZ;
        int pwmD = PWM_AVANCE - (erreur * kp);
        int pwmG = PWM_AVANCE + (erreur * kp);
        pwmD = constrain(pwmD, 0, 255);
        pwmG = constrain(pwmG, 0, 255);
        analogWrite(IN_2_D, pwmD);
        analogWrite(IN_1_G, pwmG);

        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * 0.01;
        delay(10);
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    delay(150);
}

void pivoterDroiteGyro(float angleCibleDeg) {
    angleZ = 0;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);
    analogWrite(IN_2_D, 0);
    analogWrite(IN_1_D, PWM_PIVOT + PWM_OFFSET_D);
    analogWrite(IN_1_G, PWM_PIVOT);
    analogWrite(IN_2_G, 0);

    unsigned long tPrecedent = micros();
    float gyroMax = 0;

    while (abs(angleZ) < angleCibleDeg - TOLERANCE_GYRO && !stopRequest) {
        unsigned long tMaintenant = micros();
        float dt = (tMaintenant - tPrecedent) / 1000000.0;
        tPrecedent = tMaintenant;
        float gyroZ = -readFloatGyroZ();
        if (abs(gyroZ) > abs(gyroMax)) gyroMax = gyroZ;
        angleZ += gyroZ * dt;
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    delay(200);
}

void pivoterGaucheGyro(float angleCibleDeg) {
    angleZ = 0;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);
    analogWrite(IN_1_D, 0);
    analogWrite(IN_2_D, PWM_PIVOT + PWM_OFFSET_D);
    analogWrite(IN_2_G, PWM_PIVOT);
    analogWrite(IN_1_G, 0);

    unsigned long tPrecedent = micros();
    while (abs(angleZ) < angleCibleDeg - TOLERANCE_GYRO && !stopRequest) {
        unsigned long tMaintenant = micros();
        float dt = (tMaintenant - tPrecedent) / 1000000.0;
        tPrecedent = tMaintenant;
        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * dt;
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    delay(200);
}

// ============================================================
// CERCLE (identique, avec stopRequest)
// ============================================================
void dessinerCercle(float rayonCM) {
    int pwmG, pwmD;
    if      (rayonCM <= 13) { pwmG = 100; pwmD = 0;  }
    else if (rayonCM <= 14) { pwmG = 100; pwmD = 35; }
    else if (rayonCM <= 15) { pwmG = 100; pwmD = 70; }
    else if (rayonCM <= 16) { pwmG = 100; pwmD = 75; }
    else if (rayonCM <= 17) { pwmG = 100; pwmD = 80; }
    else if (rayonCM <= 18) { pwmG = 100; pwmD = 85; }
    else if (rayonCM <= 19) { pwmG = 100; pwmD = 90; }
    else                    { pwmG = 100; pwmD = 95; }

    angleZ = 0;
    unsigned long tPrecedent = micros();
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);
    analogWrite(IN_1_G, pwmG);
    analogWrite(IN_2_G, 0);
    analogWrite(IN_2_D, pwmD);
    analogWrite(IN_1_D, 0);

    while (abs(angleZ) < 360.0 - TOLERANCE_GYRO && !stopRequest) {
        unsigned long tMaintenant = micros();
        float dt = (tMaintenant - tPrecedent) / 1000000.0;
        tPrecedent = tMaintenant;
        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * dt;
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
}

// ============================================================
// ANGLE DROIT (identique, avec stopRequest)
// ============================================================
void avancerCransAvecPID(int i) {
    countG = 0;
    countD = 0;
    angleAvance = 0;
    pidAvance_erreurPrec = 0;
    pidAvance_integral = 0;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);

    int pwmD;
    if      (i < 3) pwmD = 120;
    else if (i < 9) pwmD = 140;
    else            pwmD = 160;
    int pwmG = PWM_AVANCE_G;

    int crans;
    if      (i < 5)  crans = CRANS_AVANCE;
    else if (i < 9)  crans = 6;
    else if (i < 12) crans = 8;
    else if (i < 15) crans = 10;
    else             crans = 15;

    while (countG < crans && countD < crans && !stopRequest) {
        analogWrite(IN_2_D, pwmD);
        analogWrite(IN_1_G, pwmG);
        delay(2);
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
}

void fairAngleDroit() {
    fonctionEnCours = true;
    Serial.println("=== Debut angle droit ===");

    countG = 0;
    countD = 0;
    angleZ = 0;
    long cransCible = mmEnCrans(LONGUEUR_PREMIER_TRAIT_MM);
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);

    while (countG < cransCible && countD < cransCible && !stopRequest) {
        float erreur = angleZ;
        int pwmD = PWM_AVANCE_D - (erreur * kp);
        int pwmG = PWM_AVANCE_G + (erreur * kp);
        pwmD = constrain(pwmD, 0, 255);
        pwmG = constrain(pwmG, 0, 255);
        analogWrite(IN_2_D, pwmD);
        analogWrite(IN_1_G, pwmG);
        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * 0.01;
        delay(10);
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    delay(300);

    if (stopRequest) { fonctionEnCours = false; return; }

    int nbPas = (int)(ANGLE_TOTAL_DEG / ANGLE_PAS_DEG);
    float angleActuelTotal = 0;
    for (int i = 0; i < nbPas && !stopRequest; i++) {
        angleActuelTotal += ANGLE_PAS_DEG;
        pivoterDroiteGyro(ANGLE_PAS_DEG);
        if (i == 0) { anglePremierPivot = abs(angleZ); }
        delay(3000);
        if (stopRequest) break;
        avancerCransAvecPID(i);
        delay(3200);
    }
    fonctionEnCours = false;
    Serial.println("=== Angle droit termine ! ===");
}

void roulerLentement() {
    fonctionEnCours = true;
    countG = 0;
    countD = 0;
    digitalWrite(EN_G, HIGH);
    digitalWrite(EN_D, HIGH);

    float cransParSecD = 130.0;
    float cibleD = 0;
    unsigned long tDebut = millis();
    unsigned long tPrecedent = millis();

    while (millis() - tDebut < 15000 && !stopRequest) {
        unsigned long tMaintenant = millis();
        float dt = (tMaintenant - tPrecedent) / 1000.0;
        tPrecedent = tMaintenant;
        cibleD += cransParSecD * dt;
        int errD = (int)cibleD - countD;
        int pwmD = constrain(100 + errD * 10, 0, 255);
        analogWrite(IN_1_G, 100);
        analogWrite(IN_2_D, pwmD);
        delay(2);
    }
    digitalWrite(EN_G, LOW);
    digitalWrite(EN_D, LOW);
    fonctionEnCours = false;
}

// ============================================================
// MAGNÉTOMÈTRE LIS3MDL — identique au code utilisateur
// ============================================================
void initMag() {
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(LIS3MDL_CTRL1);
    Wire.write(0x70);
    Wire.endTransmission();

    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(LIS3MDL_CTRL3);
    Wire.write(0x00);
    Wire.endTransmission();
}
void lireMag(float &mx, float &my) {
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(LIS3MDL_OUT_X_L | 0x80);
    Wire.endTransmission(false);
    Wire.requestFrom(LIS3MDL_ADDR, 6);
    int16_t x = Wire.read() | (Wire.read() << 8);
    int16_t y = Wire.read() | (Wire.read() << 8);
    Wire.read(); Wire.read();
    mx = x - magOffsetX;
    my = y - magOffsetY;
}

void calibrerMag() {
    fonctionEnCours = true;
    float minX = 99999, maxX = -99999;
    float minY = 99999, maxY = -99999;
    angleZ = 0;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);
    analogWrite(IN_2_D, 0);
    analogWrite(IN_1_D, PWM_PIVOT);
    analogWrite(IN_1_G, PWM_PIVOT);
    analogWrite(IN_2_G, 0);

    unsigned long tPrec = micros();
    while (abs(angleZ) < 360.0 - TOLERANCE_GYRO && !stopRequest) {
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0;
        tPrec = tNow;
        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * dt;

        float mx, my;
        Wire.beginTransmission(LIS3MDL_ADDR);
        Wire.write(LIS3MDL_OUT_X_L | 0x80);
        Wire.endTransmission(false);
        Wire.requestFrom(LIS3MDL_ADDR, 6);
        int16_t rx = Wire.read() | (Wire.read() << 8);
        int16_t ry = Wire.read() | (Wire.read() << 8);
        Wire.read(); Wire.read();
        mx = rx; my = ry;
        if (mx < minX) minX = mx;
        if (mx > maxX) maxX = mx;
        if (my < minY) minY = my;
        if (my > maxY) maxY = my;
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    magOffsetX = (maxX + minX) / 2.0;
    magOffsetY = (maxY + minY) / 2.0;
    fonctionEnCours = false;
    bleSendf("LOG:Calibration OK offsetX=%.0f offsetY=%.0f", magOffsetX, magOffsetY);
}

// ============================================================
// FLÈCHE NORD (identique au code utilisateur)
// ============================================================
void dessinerFlecheNord() {
    fonctionEnCours = true;
    enMarche = false;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);
    analogWrite(IN_1_G, 150);
    analogWrite(IN_2_G, 0);
    analogWrite(IN_1_D, 150);
    analogWrite(IN_2_D, 0);

    countG = 0;
    countD = 0;

    while (!stopRequest) {
        float mx, my;
        lireMag(mx, my);
        float cap = atan2(my, mx) * 180.0 / PI;
        if (cap < 0) cap += 360.0;
        if (abs(cap - 290.0) < 1.0) break;

        if (countG >= 1 || countD >= 1) {
            digitalWrite(EN_D, LOW);
            digitalWrite(EN_G, LOW);
            delay(200);
            if (stopRequest) break;
            countG = 0;
            countD = 0;
            digitalWrite(EN_D, HIGH);
            digitalWrite(EN_G, HIGH);
            analogWrite(IN_1_G, 150);
            analogWrite(IN_2_G, 0);
            analogWrite(IN_1_D, 150);
            analogWrite(IN_2_D, 0);
        }
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    angleZ = 0;
    erreurIntegrale = 0;
    erreurprecedente = 0;
    erreurDerivee = 0;
    fonctionEnCours = false;
    enMarche = true;
}

// ============================================================
// TESTS UNITAIRES (identiques au code utilisateur)
// ============================================================
struct TestResult {
    String nom;
    bool   pass;
    String valeurMesuree;
    String valeurAttendue;
    String observation;
};
#define NB_TESTS 12
TestResult resultats[NB_TESTS];

void enregistrer(int id, String nom, bool pass,
                 String mesure, String attendu, String obs) {
    resultats[id] = { nom, pass, mesure, attendu, obs };
    bleSendf("TEST:%d %s : %s | %s | %s",
             id, nom.c_str(),
             pass ? "PASS" : "FAIL",
             mesure.c_str(), obs.c_str());
}

void test_sensRotationDroite() {
    countD = 0;
    digitalWrite(EN_D, HIGH);
    analogWrite(IN_2_D, 150);
    analogWrite(IN_1_D, 0);
    delay(500);
    digitalWrite(EN_D, LOW);
    bool pass = countD > 0;
    enregistrer(0, "Sens rotation roue droite", pass,
                "countD=" + String(countD), "countD > 0",
                pass ? "Roue avance correctement" : "Roue tourne a l'envers ou bloquee");
}
void test_sensRotationGauche() {
    countG = 0;
    digitalWrite(EN_G, HIGH);
    analogWrite(IN_1_G, 150);
    analogWrite(IN_2_G, 0);
    delay(500);
    digitalWrite(EN_G, LOW);
    bool pass = countG > 0;
    enregistrer(1, "Sens rotation roue gauche", pass,
                "countG=" + String(countG), "countG > 0",
                pass ? "Roue avance correctement" : "Roue tourne a l'envers ou bloquee");
}
void test_coupureMoteur() {
    countD = 0;
    digitalWrite(EN_D, HIGH);
    analogWrite(IN_2_D, 150);
    delay(300);
    digitalWrite(EN_D, LOW);
    long countAvant = countD;
    delay(300);
    long countApres = countD;
    bool pass = (countApres - countAvant) < 5;
    enregistrer(2, "Coupure moteur EN_D", pass,
                "delta apres coupure=" + String(countApres - countAvant), "delta < 5",
                pass ? "Moteur bien coupe" : "Roue continue a tourner apres coupure");
}
void test_seuilDemarrageD() {
    int pwmSeuil = 0;
    digitalWrite(EN_D, HIGH);
    for (int pwm = 30; pwm <= 255 && !stopRequest; pwm += 5) {
        countD = 0;
        analogWrite(IN_2_D, pwm);
        analogWrite(IN_1_D, 0);
        delay(200);
        if (countD > 2) { pwmSeuil = pwm; break; }
    }
    digitalWrite(EN_D, LOW);
    bool pass = pwmSeuil > 0 && pwmSeuil < 200;
    enregistrer(3, "Seuil demarrage roue droite", pass,
                "PWM seuil=" + String(pwmSeuil), "0 < seuil < 200",
                "Seuil mesure : PWM " + String(pwmSeuil));
}
void test_seuilDemarrageG() {
    int pwmSeuil = 0;
    digitalWrite(EN_G, HIGH);
    for (int pwm = 30; pwm <= 255 && !stopRequest; pwm += 5) {
        countG = 0;
        analogWrite(IN_1_G, pwm);
        analogWrite(IN_2_G, 0);
        delay(200);
        if (countG > 2) { pwmSeuil = pwm; break; }
    }
    digitalWrite(EN_G, LOW);
    bool pass = pwmSeuil > 0 && pwmSeuil < 200;
    enregistrer(4, "Seuil demarrage roue gauche", pass,
                "PWM seuil=" + String(pwmSeuil), "0 < seuil < 200",
                "Seuil mesure : PWM " + String(pwmSeuil));
}
void test_encodeurs() {
    countG = 0; countD = 0;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);
    analogWrite(IN_2_D, 150);
    analogWrite(IN_1_G, 150);
    delay(1000);
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    bool pass = countG > 50 && countD > 50;
    enregistrer(5, "Encodeurs incrementent", pass,
                "countG=" + String(countG) + " countD=" + String(countD),
                "countG > 50 ET countD > 50",
                pass ? "Les deux encodeurs fonctionnent" : "Un encodeur ne repond pas");
}
void test_cransTour() {
    countG = 0;
    delay(5000);     // l'utilisateur a 5s pour tourner la roue
    long crans = abs(countG);
    bool pass = crans > 340 && crans < 380;
    enregistrer(6, "Crans par tour roue gauche", pass,
                "crans mesures=" + String(crans), "340 < crans < 380",
                pass ? "Encodeur calibre" : "Verifier cablage encodeur");
}
void test_gyroscope() {
    float gyroMax = 0;
    unsigned long t = millis();
    while (millis() - t < 3000 && !stopRequest) {
        float gz = abs(readFloatGyroZ());
        if (gz > gyroMax) gyroMax = gz;
        delay(10);
    }
    bool pass = gyroMax > 5.0;
    enregistrer(7, "Gyroscope actif", pass,
                "gyroMax=" + String(gyroMax, 2) + " deg/s", "gyroMax > 5 deg/s",
                pass ? "Gyroscope OK" : "Gyroscope ne repond pas");
}
void test_deriveGyro() {
    angleZ = 0;
    unsigned long t = millis();
    while (millis() - t < 10000 && !stopRequest) {
        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * 0.01;
        delay(10);
    }
    bool pass = abs(angleZ) < 2.0;
    enregistrer(8, "Derive gyroscope 10s immobile", pass,
                "derive=" + String(angleZ, 3) + " deg", "derive < 2 deg",
                pass ? "Derive acceptable" : "Derive importante, recalibrer");
}
void test_trajectoireDroite() {
    angleZ = 0;
    float angleMax = 0;
    long cransCible = mmEnCrans(1000.0);
    countG = 0; countD = 0;
    digitalWrite(EN_D, HIGH);
    digitalWrite(EN_G, HIGH);
    while (countG < cransCible && countD < cransCible && !stopRequest) {
        float erreur = angleZ;
        int pwmD = PWM_AVANCE - (erreur * kp);
        int pwmG = PWM_AVANCE + (erreur * kp);
        pwmD = constrain(pwmD, 0, 255);
        pwmG = constrain(pwmG, 0, 255);
        analogWrite(IN_2_D, pwmD);
        analogWrite(IN_1_G, pwmG);
        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * 0.01;
        if (abs(angleZ) > abs(angleMax)) angleMax = angleZ;
        delay(10);
    }
    digitalWrite(EN_D, LOW);
    digitalWrite(EN_G, LOW);
    bool pass = abs(angleZ) < 5.0;
    enregistrer(9, "Trajectoire droite 1m", pass,
                "angleZ final=" + String(angleZ, 2) + " deg", "angleZ < 5 deg",
                pass ? "Trajectoire correcte" : "Derive trop importante");
}
void test_pivot90() {
    float angles[3];
    float moyenne = 0;
    for (int essai = 0; essai < 3 && !stopRequest; essai++) {
        angleZ = 0;
        pivoterDroiteGyro(90.0);
        angles[essai] = abs(angleZ);
        moyenne += angles[essai];
        delay(2000);
    }
    moyenne /= 3.0;
    float ecartMax = 0;
    for (int i = 0; i < 3; i++) {
        float e = abs(angles[i] - moyenne);
        if (e > ecartMax) ecartMax = e;
    }
    bool pass = ecartMax < 3.0;
    String detail = "essais: " + String(angles[0],1) + " / " +
                    String(angles[1],1) + " / " + String(angles[2],1);
    enregistrer(10, "Reproductibilite pivot 90 deg", pass,
                "ecart max=" + String(ecartMax, 2) + " deg | " + detail,
                "ecart max < 3 deg",
                pass ? "Pivot reproductible" : "Pivot variable");
}
void test_cercleRetour() {
    angleZ = 0;
    dessinerCercle(20.0);
    float angleTotal = abs(angleZ);
    bool pass = angleTotal > 355.0 && angleTotal < 365.0;
    enregistrer(11, "Cercle 20cm retour point depart", pass,
                "angleZ total=" + String(angleTotal, 2) + " deg",
                "355 < angle < 365 deg",
                pass ? "Cercle complet correct" : "Cercle incomplet ou depasse");
}

// ============================================================
// EXÉCUTION DES COMMANDES BLE (équivalent des routes HTTP)
// ============================================================
void executerCommande(const String &cmd) {
    bleSendf("LOG:CMD=%s", cmd.c_str());

    if (cmd == "PING") {
        bleSend("PONG");
    }
    else if (cmd == "STATUS") {
        float mx, my;
        lireMag(mx, my);
        float cap = atan2(my, mx) * 180.0 / PI;
        if (cap < 0) cap += 360.0;
        bleSendf("STATUS:cap=%.1f,countG=%ld,countD=%ld,angleZ=%.2f,rayon=%.0f",
                 cap, countG, countD, angleZ, rayonCercle);
    }
    else if (cmd == "LED:ON")  { digitalWrite(LED_BUILTIN, HIGH); bleSend("LOG:LED ON"); }
    else if (cmd == "LED:OFF") { digitalWrite(LED_BUILTIN, LOW);  bleSend("LOG:LED OFF"); }

    else if (cmd == "ROULER") {
        digitalWrite(EN_D, HIGH);
        digitalWrite(EN_G, HIGH);
        analogWrite(IN_2_D, 60);
        analogWrite(IN_1_G, 60);
        bleSend("LOG:Rouler");
    }
    else if (cmd == "AVANCE_DROIT") {
        angleZ = 0;
        enMarche = true;
        digitalWrite(EN_D, HIGH);
        digitalWrite(EN_G, HIGH);
        bleSend("LOG:Avance droit (marche libre asservie)");
    }
    else if (cmd == "CERCLE_LENT") {
        roulerLentement();
        bleSend("DONE:CERCLE_LENT");
    }

    // === SÉQUENCES PRINCIPALES ===
    else if (cmd == "SEQ1") {
        bleSend("LOG:SEQ1 escalier debut");
        fairAngleDroit();
        bleSend(stopRequest ? "LOG:SEQ1 interrompue" : "DONE:SEQ1");
    }
    else if (cmd.startsWith("SEQ2:R")) {
        float r = cmd.substring(6).toFloat();
        if (r < 13.0) {
            bleSend("ERR:Rayon minimum 13 cm");
        } else {
            rayonCercle = r * 10.0;       // mm
            bleSendf("LOG:SEQ2 cercle r=%.1fcm", r);
            dessinerCercle(r);
            bleSend(stopRequest ? "LOG:SEQ2 interrompue" : "DONE:SEQ2");
        }
    }
    else if (cmd == "SEQ3") {
        bleSend("LOG:SEQ3 fleche Nord");
        dessinerFlecheNord();
        bleSend(stopRequest ? "LOG:SEQ3 interrompue" : "DONE:SEQ3");
    }

    // === CALIBRAGES ===
    else if (cmd == "CAL:MAG") {
        bleSend("LOG:Calibrage magneto (10s)");
        calibrerMag();
        bleSend("DONE:CAL_MAG");
    }

    // === PID ===
    else if (cmd.startsWith("PID:")) {
        int iP = cmd.indexOf("KP"), iI = cmd.indexOf("KI"), iD = cmd.indexOf("KD");
        if (iP > 0 && iI > 0 && iD > 0) {
            kp = cmd.substring(iP+2, cmd.indexOf(':', iP)).toFloat();
            ki = cmd.substring(iI+2, cmd.indexOf(':', iI)).toFloat();
            kd = cmd.substring(iD+2).toFloat();
            bleSendf("LOG:PID Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
        }
    }

    // === TESTS UNITAIRES ===
    else if (cmd.startsWith("TEST:")) {
        int id = cmd.substring(5).toInt();
        bleSendf("LOG:Lancement TEST %d", id);
        switch (id) {
            case 0:  test_sensRotationDroite();  break;
            case 1:  test_sensRotationGauche();  break;
            case 2:  test_coupureMoteur();       break;
            case 3:  test_seuilDemarrageD();     break;
            case 4:  test_seuilDemarrageG();     break;
            case 5:  test_encodeurs();           break;
            case 6:  test_cransTour();           break;
            case 7:  test_gyroscope();           break;
            case 8:  test_deriveGyro();          break;
            case 9:  test_trajectoireDroite();   break;
            case 10: test_pivot90();             break;
            case 11: test_cercleRetour();        break;
            default: bleSendf("ERR:TEST id %d inconnu", id); return;
        }
        bleSendf("DONE:TEST%d", id);
    }

    // === JOYSTICK (équivalent du WebSocket d'origine) ===
    // Format : JOY:LY<x>:L2<y>:R2<z>  (valeurs entre -1 et 1)
    else if (cmd.startsWith("JOY:")) {
        int iLy = cmd.indexOf("LY"), iL2 = cmd.indexOf("L2"), iR2 = cmd.indexOf("R2");
        if (iLy > 0 && iL2 > 0 && iR2 > 0) {
            ly = cmd.substring(iLy+2, cmd.indexOf(':', iLy)).toFloat();
            l2 = cmd.substring(iL2+2, cmd.indexOf(':', iL2)).toFloat();
            r2 = cmd.substring(iR2+2).toFloat();
            if (!enMarche) {
                digitalWrite(EN_D, HIGH);
                digitalWrite(EN_G, HIGH);
                analogWrite(IN_2_D, r2 * 255 - ly * 128 * r2);
                analogWrite(IN_1_G, r2 * 255 + ly * 128 * r2);
                analogWrite(IN_1_D, l2 * 255);
                analogWrite(IN_2_G, l2 * 255);
            }
        }
    }

    else {
        bleSendf("ERR:UNKNOWN=%s", cmd.c_str());
    }
}

// ============================================================
// CALLBACK BLE (réception des commandes)
// ============================================================
class CmdCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String data = pChar->getValue().c_str();
        // Parse ligne par ligne
        int start = 0, end;
        while ((end = data.indexOf('\n', start)) >= 0) {
            String c = data.substring(start, end);
            c.trim();
            if (c.length() > 0) {
                if (c == "STOP") {
                    // Arrêt IMMÉDIAT depuis le callback (non bloquant)
                    stopRequest = true;
                    stopper_immediat();
                    bleSend("LOG:STOPPED");
                } else {
                    // Autres commandes : on les passe à loop()
                    commandePendingStr = c;
                    commandePending = true;
                }
            }
            start = end + 1;
        }
        if (start < (int)data.length()) {
            String c = data.substring(start); c.trim();
            if (c.length() > 0) {
                if (c == "STOP") {
                    stopRequest = true;
                    stopper_immediat();
                    bleSend("LOG:STOPPED");
                } else {
                    commandePendingStr = c;
                    commandePending = true;
                }
            }
        }
    }
};

// ============================================================
// TÉLÉMÉTRIE PÉRIODIQUE (compatible interface React)
// ============================================================
unsigned long last_tlm_ms = 0;
void send_telemetry() {
    if (millis() - last_tlm_ms < 100) return;        // 10 Hz suffit
    last_tlm_ms = millis();
    if (!bleConnected) return;

    float mx, my;
    lireMag(mx, my);
    float cap = atan2(my, mx) * 180.0 / PI;
    if (cap < 0) cap += 360.0;

    // Format compatible interface : TLM,t,cap,x,y,vG,vD
    // → on mappe : cap=cap, x=countG, y=countD, vG=angleZ, vD=rayonCercle
    bleSendf("TLM,%lu,%.1f,%ld,%ld,%.2f,%.0f",
             millis(), cap, countG, countD, angleZ, rayonCercle);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n[BOOT] Drawbot BLE — code fonctionnel utilisateur");

    pinMode(LEDU1, OUTPUT);
    pinMode(LEDU2, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(IN_1_D, OUTPUT);
    pinMode(IN_2_D, OUTPUT);
    pinMode(EN_D, OUTPUT);
    pinMode(IN_1_G, OUTPUT);
    pinMode(IN_2_G, OUTPUT);
    pinMode(EN_G, OUTPUT);
    pinMode(ENC_G_CH_A, INPUT_PULLUP);
    pinMode(ENC_G_CH_B, INPUT_PULLUP);
    pinMode(ENC_D_CH_A, INPUT_PULLUP);
    pinMode(ENC_D_CH_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), isrG_A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), isrD_A, RISING);

    Wire.begin(SDA, SCL);
    Wire.setClock(400000);
    initIMU();
    initMag();

    // === Init BLE ===
    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(247);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new DrawbotServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCmdChar = pService->createCharacteristic(
        CMD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pCmdChar->setCallbacks(new CmdCharCallbacks());
    pTlmChar = pService->createCharacteristic(
        TLM_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pTlmChar->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->setMinPreferred(0x06);
    pAdv->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising " DEVICE_NAME);
    digitalWrite(LEDU1, HIGH);
    bleSend("READY");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    // 1. Exécuter une commande BLE en attente
    if (commandePending) {
        commandePending = false;
        stopRequest = false;                  // reset avant chaque commande
        String cmd = commandePendingStr;
        executerCommande(cmd);
    }

    // 2. PID en marche libre (identique au code utilisateur d'origine)
    if (!fonctionEnCours) {
        static unsigned long tPrec = micros();
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0;
        tPrec = tNow;

        float gyroZ = -readFloatGyroZ();
        angleZ += gyroZ * dt;

        if (enMarche) {
            float erreur = angleZ;
            erreurIntegrale += erreur;
            erreurIntegrale = constrain(erreurIntegrale, -50, 50);
            erreurDerivee = erreur - erreurprecedente;
            erreurprecedente = erreur;
            int pwmD = VITESSE - (erreur * kp + erreurDerivee * kd + erreurIntegrale * ki);
            int pwmG = VITESSE + (erreur * kp + erreurDerivee * kd + erreurIntegrale * ki);
            pwmD = constrain(pwmD, 0, 255);
            pwmG = constrain(pwmG, 0, 255);
            analogWrite(IN_2_D, pwmD);
            analogWrite(IN_1_G, pwmG);
        }
    }

    // 3. Télémétrie périodique (10 Hz)
    send_telemetry();
}