
#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#define LEDU1 25
#define LEDU2 26
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#define EN_D 23          // Enable driver roue droite
#define EN_G 4           // Enable driver roue gauche

#define IN_1_D 19        // Roue droite  - sens 1 (recul)
#define IN_2_D 18        // Roue droite  - sens 2 (avance)
#define IN_1_G 17        // Roue gauche  - sens 1 (avance)
#define IN_2_G 16        // Roue gauche  - sens 2 (recul)

#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14

#define SDA 21
#define SCL 22

#define CRANS_PAR_TOUR    360      // crans d'encodeur par tour de roue (mesuré)
#define DIAMETRE_ROUE_MM  65.0     // diamètre roulant effectif (mesuré)
#define ECARTEMENT_ROUES_MM 88.0   // entraxe roue gauche <-> roue droite

#define CIRCONF_MM   (PI * DIAMETRE_ROUE_MM)
#define CRANS_PAR_MM (CRANS_PAR_TOUR / CIRCONF_MM)   // ~1.76 cran/mm

#define PWM_AVANCE       100   // PWM de base en ligne droite
#define PWM_PIVOT        120   // PWM de base en pivot sur place
#define PWM_OFFSET_D      20   // compensation roue droite en pivot
#define PWM_CERCLE_BASE  120   // PWM roue extérieure pour les cercles
#define TOLERANCE_GYRO   0.7   // marge d'arrêt gyro (°)
#define RAYON_VIRAGE_CM  8.0   // rayon des virages (plus grand = plus lisse)
#define PWM_VIRAGE_EXT   130   // PWM roue exterieure pendant un virage en arc
#define PWM_VIRAGE_MIN    55   // PWM mini roue interieure (rotation continue = lisse)

float kp = 5.0;
float ki = 0.0;
float kd = 0.5;
float kpPos = 6.0;
float kpCercle = 8.0;

// Réglages anti-boucles pour SEQ1 : on pilote les virages avec les encodeurs,
// pas avec le gyro. Cela évite les grandes courbes quand le gyro dérive.
float kpSync       = 0.8;   // synchro roues en ligne droite (commande SYNC:x)
float kpPivotSync  = 2.5;   // synchro roues en pivot
float pivotCal     = 1.0;   // calibration angle pivot (commande PIVOTCAL:x)
int   pwmAvanceSeq = 75;    // vitesse lente pour les traits droits (commande VAVANCE:x)
int   pwmPivotSeq  = 75;    // vitesse lente pour les virages (commande VPIVOT:x)

// Calibration réelle distance/angle par encodeurs.
// Ton test montre : commande 20 cm -> résultat ≈ 12 cm.
// Donc il faut demander environ 20/12 = 1.67 fois plus de crans.
// Réglable sans recompiler : DISTCAL:1.67 ou CALFWD:20:12
float distCal = 1.67;

// Suivi de trajectoire (escalier) : correction proportionnelle
float kpLigneCap   = 60.0;   // PWM par radian d'erreur de CAP
float kpLigneCross = 0.8;    // PWM par mm d'erreur LATERALE (revenir au centre)
float suiviFwdMin  = 0.25;   // fraction de vitesse mini en virage (plus petit = vire + serre)

#define LIS3MDL_ADDR     0x1E
#define LIS3MDL_CTRL1    0x20
#define LIS3MDL_CTRL3    0x22
#define LIS3MDL_OUT_X_L  0x28

float magOffsetX = 0;   // offsets "hard-iron" (remplis par CAL_MAG)
float magOffsetY = 0;

// Cap (lecture magnéto, en °) que renvoie le capteur quand le robot
// regarde le Nord. Calibré par CAL_NORD ; valeur de départ ci-dessous.
float capNordReference = 290.0;

// ============================================================
//  CENTRALE INERTIELLE LSM6DS3 (I2C brut — gyroscope Z)
// ============================================================
#define LSM6DS3_ADDR     0x6B
#define LSM6DS3_WHO_AM_I 0x0F
#define LSM6DS3_CTRL1_XL 0x10
#define LSM6DS3_CTRL2_G  0x11
#define LSM6DS3_CTRL3_C  0x12
#define LSM6DS3_OUTZ_L_G 0x26

void initIMU() {
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(LSM6DS3_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM6DS3_ADDR, 1);
    uint8_t who = Wire.read();
    Serial.printf("[IMU] WHO_AM_I = 0x%02X\n", who);

    Wire.beginTransmission(LSM6DS3_ADDR);   // XL 416 Hz, ±2g
    Wire.write(LSM6DS3_CTRL1_XL); Wire.write(0x60);
    Wire.endTransmission();

    Wire.beginTransmission(LSM6DS3_ADDR);   // Gyro 416 Hz, ±245 dps
    Wire.write(LSM6DS3_CTRL2_G); Wire.write(0x64);
    Wire.endTransmission();

    Wire.beginTransmission(LSM6DS3_ADDR);   // BDU + auto-increment
    Wire.write(LSM6DS3_CTRL3_C); Wire.write(0x44);
    Wire.endTransmission();
}

// Vitesse angulaire Z en °/s
float readFloatGyroZ() {
    Wire.beginTransmission(LSM6DS3_ADDR);
    Wire.write(LSM6DS3_OUTZ_L_G);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM6DS3_ADDR, 2);
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    int16_t raw = (int16_t)((hi << 8) | lo);
    return raw * 0.0175f;    // ±500 dps -> 17.5 mdps/LSB (evite la saturation en pivot)
}

void initMag() {
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(LIS3MDL_CTRL1); Wire.write(0x70);   // haute perf, 10 Hz
    Wire.endTransmission();
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(LIS3MDL_CTRL3); Wire.write(0x00);   // mode continu
    Wire.endTransmission();
}

void lireMag(float &mx, float &my) {
    Wire.beginTransmission(LIS3MDL_ADDR);
    Wire.write(LIS3MDL_OUT_X_L | 0x80);            // auto-incrément
    Wire.endTransmission(false);
    Wire.requestFrom(LIS3MDL_ADDR, 6);
    int16_t x = Wire.read() | (Wire.read() << 8);
    int16_t y = Wire.read() | (Wire.read() << 8);
    Wire.read(); Wire.read();                       // Z ignoré
    mx = x - magOffsetX;
    my = y - magOffsetY;
}

// Cap magnétique calibré (0..360°)
float capActuel() {
    float mx, my; lireMag(mx, my);
    float c = atan2(my, mx) * 180.0 / PI;
    if (c < 0) c += 360.0;
    return c;
}

// Plus court écart signé entre deux caps (-180..180)
float ecartCap(float cible, float courant) {
    float d = cible - courant;
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

// ============================================================
//  ÉTAT GLOBAL
// ============================================================
volatile long countG = 0;       // crans encodeur gauche
volatile long countD = 0;       // crans encodeur droit
volatile bool stopRequest = false;   // demande d'arrêt (callback BLE)

bool  fonctionEnCours   = false;     // true = une séquence tourne
bool  enMarche          = false;     // mode "avance libre" asservie en cap
float angleZ            = 0;         // cap relatif intégré (gyro), en °
float anglePremierPivot = 0;         // mémorisé pour la télémétrie/SEQ1
float rayonCercle       = 0;         // dernier rayon demandé (mm)

// Repère cartésien (odométrie) : pose de l'axe des roues
float poseX = 0, poseY = 0, poseTh = 0;   // x,y en mm ; theta en rad
long  odoLastG = 0, odoLastD = 0;

// Joystick (mode pilotage manuel)
float jLY = 0, jL2 = 0, jR2 = 0;

// Commande en attente (transmise du callback BLE vers loop())
volatile bool commandePending = false;
String        commandePendingStr = "";


void IRAM_ATTR isrG_A() {
    if (digitalRead(ENC_G_CH_B) == HIGH) countG--;
    else                                  countG++;
}
void IRAM_ATTR isrD_A() {
    if (digitalRead(ENC_D_CH_B) == HIGH) countD++;
    else                                  countD--;
}


#define DEVICE_NAME   "Drawbot_ECE"
#define SERVICE_UUID  "0000ace0-1234-5678-1234-56789abcdef0"
#define CMD_CHAR_UUID "0000ace1-1234-5678-1234-56789abcdef0"
#define TLM_CHAR_UUID "0000ace2-1234-5678-1234-56789abcdef0"

BLEServer*         pServer  = nullptr;
BLECharacteristic* pCmdChar = nullptr;
BLECharacteristic* pTlmChar = nullptr;
volatile bool      bleConnected = false;

void bleSend(const String &s) {
    Serial.println(s);
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

// Télémétrie : TLM,millis,countG,countD,err,angleZ,anglePivot,rayon,cap,mx,my,ox,oy
unsigned long lastTlm = 0;
void envoyerTelemetrie(bool force = false) {
    if (!force && millis() - lastTlm < 180) return;
    lastTlm = millis();
    if (!bleConnected) return;
    float mx, my; lireMag(mx, my);
    float cap = atan2(my, mx) * 180.0 / PI;
    if (cap < 0) cap += 360.0;
    bleSendf("TLM,%lu,%ld,%ld,%ld,%.2f,%.2f,%.0f,%.1f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.1f",
             millis(), countG, countD, (countD - countG),
             angleZ, anglePremierPivot, rayonCercle,
             cap, mx, my, magOffsetX, magOffsetY,
             poseX, poseY, poseTh * 180.0 / PI);
}

// ============================================================
//  COMMANDE BAS NIVEAU DES MOTEURS (PWM signé)
//  +pwm = avance, -pwm = recul. Remet toujours l'autre demi-pont à 0.
// ============================================================
void enableMoteurs(bool on) {
    digitalWrite(EN_D, on ? HIGH : LOW);
    digitalWrite(EN_G, on ? HIGH : LOW);
}
void motG(int pwm) {                         // roue gauche
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) { analogWrite(IN_2_G, 0);   analogWrite(IN_1_G, pwm); }
    else          { analogWrite(IN_1_G, 0);   analogWrite(IN_2_G, -pwm); }
}
void motD(int pwm) {                         // roue droite
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) { analogWrite(IN_1_D, 0);   analogWrite(IN_2_D, pwm); }
    else          { analogWrite(IN_2_D, 0);   analogWrite(IN_1_D, -pwm); }
}
void moteursStop() {
    motG(0); motD(0);
    enableMoteurs(false);
}
void stopperImmediat() {            // appelé depuis le callback BLE (STOP)
    enMarche = false;
    fonctionEnCours = false;
    moteursStop();
}
long mmEnCrans(float mm) { return (long)(mm * CRANS_PAR_MM * distCal); }

// ---- ODOMÉTRIE : met à jour la pose cartésienne (x,y,theta) ----
void resyncOdo() { odoLastG = countG; odoLastD = countD; }
void resetPose() { poseX = 0; poseY = 0; poseTh = 0; resyncOdo(); }
void majOdometrie() {
    long g = countG, d = countD;
    float dl = (g - odoLastG) / (float)(CRANS_PAR_MM * distCal);   // mm roue gauche calibrée
    float dr = (d - odoLastD) / (float)(CRANS_PAR_MM * distCal);   // mm roue droite calibrée
    odoLastG = g; odoLastD = d;
    float dc  = 0.5f * (dl + dr);                      // avance du centre
    float dth = (dr - dl) / ECARTEMENT_ROUES_MM;       // rotation (rad)
    poseTh += dth;
    poseX  += dc * cosf(poseTh);
    poseY  += dc * sinf(poseTh);
}

// Avance en ligne droite de distanceMM (>0 avant, <0 arrière),
// corrigée par les ENCODEURS uniquement.
// Objectif : éviter les grandes courbes si le gyro a un biais.
void avancerMM(float distanceMM) {
    long cible  = labs(mmEnCrans(distanceMM));
    int  sens   = (distanceMM >= 0) ? 1 : -1;
    long startG = countG, startD = countD;
    angleZ = 0;                              // seulement pour la télémétrie
    enableMoteurs(true);
    unsigned long tPrec = micros();

    while (!stopRequest) {
        majOdometrie();
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0; tPrec = tNow;
        angleZ += (-readFloatGyroZ()) * dt;   // mesure, pas commande

        long dg = labs(countG - startG);
        long dd = labs(countD - startD);
        long avg = (dg + dd) / 2;
        if (avg >= cible) break;

        long reste = cible - avg;
        int base = constrain(pwmAvanceSeq, 55, 180);
        long zoneRalent = max(1L, mmEnCrans(60.0));
        if (reste < zoneRalent) {
            base = 55 + (int)((base - 55) * (reste / (float)zoneRalent));
            base = constrain(base, 55, pwmAvanceSeq);
        }

        // Si la gauche a parcouru plus que la droite, on ralentit G et on accélère D.
        int corr = (int)((dg - dd) * kpSync);
        int pwmG = sens * constrain(base - corr, 45, 200);
        int pwmD = sens * constrain(base + corr, 45, 200);

        motG(pwmG);
        motD(pwmD);
        envoyerTelemetrie();
        delay(6);
    }
    moteursStop();
    delay(120);
}

// Pivot sur place corrigé : arrêt sur ENCODEURS, pas sur gyro.
// angleDeg > 0 : gauche ; angleDeg < 0 : droite.
// Pour régler l'angle : PIVOTCAL:0.90 si le robot tourne trop, PIVOTCAL:1.10 s'il ne tourne pas assez.
void pivoter(float angleDeg) {
    int sens = (angleDeg >= 0) ? +1 : -1;        // +1 = gauche
    long startG = countG, startD = countD;
    float arcMM = (fabs(angleDeg) * PI / 180.0) * (ECARTEMENT_ROUES_MM / 2.0) * pivotCal;
    long cible = labs(mmEnCrans(arcMM));
    angleZ = 0;                                  // télémétrie uniquement
    enableMoteurs(true);
    unsigned long tPrec = micros();

    while (!stopRequest) {
        majOdometrie();
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0; tPrec = tNow;
        angleZ += (-readFloatGyroZ()) * dt;

        long dg = labs(countG - startG);
        long dd = labs(countD - startD);
        long avg = (dg + dd) / 2;
        if (avg >= cible) break;

        long reste = cible - avg;
        int base = constrain(pwmPivotSeq, 55, 170);
        long zoneRalent = max(1L, mmEnCrans(20.0));
        if (reste < zoneRalent) {
            base = 55 + (int)((base - 55) * (reste / (float)zoneRalent));
            base = constrain(base, 55, pwmPivotSeq);
        }

        // Les deux roues doivent faire le même nombre de crans en sens opposé.
        int corr = (int)((dg - dd) * kpPivotSync);
        int pwmG = constrain(base - corr, 50, 180);
        int pwmD = constrain(base + corr, 50, 180);

        // gauche : roue gauche recule, roue droite avance ; droite : inverse.
        motG(-sens * pwmG);
        motD(+sens * pwmD);
        envoyerTelemetrie();
        delay(5);
    }
    moteursStop();
    delay(150);
    bleSendf("LOG:PIVOT cible=%ld G=%ld D=%ld gyro=%.1f cal=%.2f", cible,
             labs(countG - startG), labs(countD - startD), angleZ, pivotCal);
}
inline void pivoterGauche(float a) { pivoter(+fabs(a)); }
inline void pivoterDroite(float a) { pivoter(-fabs(a)); }


void asservPosition(float distanceMM) {
    long cible = mmEnCrans(distanceMM);
    countG = 0; countD = 0; resyncOdo();
    enableMoteurs(true);
    unsigned long t0 = millis();
    // Régule jusqu'à atteindre la consigne, puis maintient la position 1,5 s
    while (!stopRequest && millis() - t0 < 5000) {
        long eG = cible - countG;
        long eD = cible - countD;
        int pwmG = constrain((int)(kpPos * eG), -200, 200);
        int pwmD = constrain((int)(kpPos * eD), -200, 200);
        // Petit seuil pour vaincre le frottement statique
        if (abs(eG) > 1 && abs(pwmG) < 55) pwmG = (pwmG >= 0 ? 55 : -55);
        if (abs(eD) > 1 && abs(pwmD) < 55) pwmD = (pwmD >= 0 ? 55 : -55);
        if (abs(eG) <= 1) pwmG = 0;
        if (abs(eD) <= 1) pwmD = 0;
        motG(pwmG); motD(pwmD);
        envoyerTelemetrie();
        delay(5);
    }
    moteursStop();
}

// ============================================================
//  SÉQUENCE 1 — "ESCALIER"
//  20 cm tout droit, 90° gauche, 10 cm, 90° droite, 40 cm
// ============================================================
void marquerPoint() {
    // Petit repère orthogonal (~1 cm) pour rendre départ/arrivée identifiables
    pivoterDroite(90);
    avancerMM(10);
    avancerMM(-10);
    pivoterGauche(90);
}
// ============================================================
//  VIRAGE EN ARC (courbe lisse) — comme fairAngleDroit du code qui marche
//  Le robot tourne EN AVANCANT (roue ext. + vite, roue int. - vite) -> le
//  stylo a l'avant trace un bel arc au lieu d'une boucle serree.
//  Arret sur le GYRO -> angle exact. Suivi du ratio par ENCODEURS (marche
//  meme a petit rayon, comme le cercle). sens = +1 gauche, -1 droite.
// ============================================================
void tournerArc(float angleDeg, float rayonCM, int sens) {
    float R     = rayonCM * 10.0;
    float L     = ECARTEMENT_ROUES_MM;
    float ratio = (R - L / 2.0) / (R + L / 2.0);   // vitesse interieur / exterieur
    bool  gaucheInterieure = (sens > 0);           // tourne a gauche -> roue G interieure

    // BOUCLE OUVERTE : PWM constants sur les 2 roues -> mouvement tres LISSE
    // (pas de pulsation de la roue interieure). L'angle reste exact (arret gyro).
    int pwmExt = PWM_VIRAGE_EXT;
    int pwmInt = (int)(pwmExt * ratio);
    if (pwmInt < PWM_VIRAGE_MIN) pwmInt = PWM_VIRAGE_MIN;   // roue int. tjs en rotation
    pwmInt = constrain(pwmInt, 0, 255);

    angleZ = 0;
    enableMoteurs(true);
    unsigned long tPrec = micros();
    while (fabs(angleZ) < fabs(angleDeg) - TOLERANCE_GYRO && !stopRequest) {
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0; tPrec = tNow;
        angleZ += (-readFloatGyroZ()) * dt;
        majOdometrie();
        if (gaucheInterieure) { motD(pwmExt); motG(pwmInt); }   // ext = roue droite
        else                  { motG(pwmExt); motD(pwmInt); }   // ext = roue gauche
        envoyerTelemetrie();
        delay(4);
    }
    moteursStop();
    delay(120);
}

// ============================================================
//  SUIVI DE TRAJECTOIRE — correction proportionnelle (Kp)
//  Le robot suit la ligne A->B en corrigeant : erreur de CAP + erreur
//  LATERALE (distance a la ligne). Au virage il tourne vers la nouvelle
//  ligne SANS depasser, puis revient au centre. (Pas d'arc impose.)
// ============================================================
void suivreSegment(float ax, float ay, float bx, float by) {
    float dx = bx - ax, dy = by - ay;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 1.0) return;
    float ux = dx/len, uy = dy/len;             // direction de la ligne
    float capCible = atan2f(uy, ux);            // cap desire (rad)
    enableMoteurs(true);

    while (!stopRequest) {
        majOdometrie();
        float px = poseX - ax, py = poseY - ay;
        float along = px*ux + py*uy;            // avancement le long de la ligne
        float cross = -px*uy + py*ux;           // erreur laterale (>0 = a gauche)
        if (along >= len) break;                // bout du segment atteint

        float capErr = capCible - poseTh;       // erreur de cap
        while (capErr >  PI) capErr -= 2.0*PI;
        while (capErr < -PI) capErr += 2.0*PI;

        // correction : cap + recentrage lateral. corr>0 => tourner a gauche
        float corr = kpLigneCap * capErr - kpLigneCross * cross;
        // on ralentit quand il faut tourner fort -> vire serre, pas de depassement
        float fwd  = PWM_AVANCE * fmaxf(suiviFwdMin, cosf(capErr));

        int pwmD = (int)(fwd + corr);           // tourner a gauche => roue D + vite
        int pwmG = (int)(fwd - corr);
        motD(constrain(pwmD, -255, 255));
        motG(constrain(pwmG, -255, 255));
        envoyerTelemetrie();
        delay(8);
    }
}

void sequenceEscalier() {
    fonctionEnCours = true;
    resetPose();
    bleSend("LOG:SEQ1 escalier - debut (distCal applique; virages = pivot sur place)");
    // Segments DROITS tenus par le gyro + virages 90 sur place (gyro).
    // -> traits bien droits ; seul le coin est legerement arrondi (stylo avant).
    avancerMM(200);     if (stopRequest) { moteursStop(); fonctionEnCours=false; return; }  // 20 cm
    pivoterGauche(90);  if (stopRequest) { moteursStop(); fonctionEnCours=false; return; }  // 90 gauche
    anglePremierPivot = poseTh * 180.0 / PI;
    avancerMM(100);     if (stopRequest) { moteursStop(); fonctionEnCours=false; return; }  // 10 cm
    pivoterDroite(90);  if (stopRequest) { moteursStop(); fonctionEnCours=false; return; }  // 90 droite
    avancerMM(400);     if (stopRequest) { moteursStop(); fonctionEnCours=false; return; }  // 40 cm
    moteursStop();
    fonctionEnCours = false;
    bleSendf("DONE:SEQ1 pose=(%.0f,%.0f) theta=%.1f", poseX, poseY, poseTh*180.0/PI);
}

// ============================================================
//  SÉQUENCE 2 — "CERCLE" (modèle cinématique différentiel)
//  Rayon du centre = R ; roue ext = R+L/2 ; roue int = R-L/2.
//  Pour R < L/2 la roue intérieure recule (cercle serré).
//  Le stylo doit être proche de l'axe des roues pour les petits rayons.
// ============================================================
void dessinerCercle(float rayonCM, int sens = +1) {
    // sens = +1 : sens anti-horaire (tourne à gauche) -> roue gauche intérieure
    fonctionEnCours = true;
    rayonCercle = rayonCM * 10.0;
    float R = rayonCM * 10.0;
    float L = ECARTEMENT_ROUES_MM;
    float arcExt = 2.0 * PI * (R + L / 2.0);     // mm  (>0)
    float arcInt = 2.0 * PI * (R - L / 2.0);     // mm  (signé)
    long  tgtExt = mmEnCrans(arcExt);
    float ratio  = arcInt / arcExt;              // intérieur / extérieur

    countG = 0; countD = 0; angleZ = 0; resyncOdo();
    enableMoteurs(true);
    unsigned long tPrec = micros();

    // Extérieure = roue droite (sens horaire of robot), Intérieure = gauche,
    // selon "sens". On garde l'extérieure à PWM ~constant, l'intérieure suit.
    bool gaucheInterieure = (sens > 0);

    while (!stopRequest) {
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0; tPrec = tNow;
        angleZ += (-readFloatGyroZ()) * dt;

        long cntExt = gaucheInterieure ? countD : countG;
        long cntInt = gaucheInterieure ? countG : countD;
        if (labs(cntExt) >= labs(tgtExt) || fabs(angleZ) >= 360.0) break;

        // L'intérieure doit suivre le ratio par rapport à l'extérieure
        long cibleInt = (long)(cntExt * ratio);
        long err      = cibleInt - cntInt;
        int  pwmExt   = PWM_CERCLE_BASE;
        int  pwmInt   = (int)(PWM_CERCLE_BASE * ratio + kpCercle * err);
        pwmInt = constrain(pwmInt, -255, 255);

        if (gaucheInterieure) { motD(pwmExt);  motG(pwmInt); }   // extérieure droite
        else                  { motG(pwmExt);  motD(pwmInt); }   // extérieure gauche
        envoyerTelemetrie();
        delay(4);
    }
    moteursStop();
    fonctionEnCours = false;
}

// ============================================================
//  SÉQUENCE 3 — "NORD" : oriente vers le Nord puis dessine une flèche
//  à pointe pleine (triangle rempli par triangles concentriques).
// ============================================================
void orienterNord() {
    // Pivote par petits pas vers la droite jusqu'à viser le Nord (±5°).
    // En tournant dans un seul sens on passe forcément par le Nord en <1 tour.
    for (int i = 0; i < 130 && !stopRequest; i++) {
        if (fabs(ecartCap(capNordReference, capActuel())) < 5.0) break;
        pivoterDroite(3.0);
        delay(40);
        envoyerTelemetrie();
    }
}
void trianglePlein(float coteMM) {
    // Triangle équilatéral rempli par 4 contours concentriques décroissants.
    // Le stylo restant baissé, les traits de liaison restent à l'intérieur.
    int   nbContours = 4;
    float cote = coteMM;
    float pas  = coteMM / (nbContours + 1);
    for (int k = 0; k < nbContours && !stopRequest; k++) {
        for (int s = 0; s < 3 && !stopRequest; s++) {
            avancerMM(cote);
            pivoterGauche(120);          // angle extérieur d'un triangle
        }
        // rentrer un peu vers le centre avant le contour suivant
        avancerMM(pas);
        pivoterGauche(30);
        cote -= pas;
        if (cote < pas) break;
    }
}
void sequenceNord() {
    fonctionEnCours = true;
    bleSend("LOG:SEQ3 - recherche du Nord");
    orienterNord();                 if (stopRequest) { fonctionEnCours=false; return; }
    bleSend("LOG:SEQ3 - oriente, trace de la fleche");
    avancerMM(45);                  // hampe de la flèche (>3 cm avec la pointe)
    if (!stopRequest) trianglePlein(30);   // pointe pleine
    moteursStop();
    fonctionEnCours = false;
    bleSend("DONE:SEQ3");
}

// ============================================================
//  FA1 — Suite de carrés en spirale (longueur + nombre paramétrables)
// ============================================================
void faCarresSpirale(float premierCoteCM, int nbCarres) {
    fonctionEnCours = true;
    bleSendf("LOG:FA1 spirale L=%.1fcm n=%d", premierCoteCM, nbCarres);
    float cote = premierCoteCM * 10.0;
    float pas  = premierCoteCM * 10.0 * 0.6;   // élargissement par tour
    int   nbCotes = 4 * nbCarres;
    for (int i = 0; i < nbCotes && !stopRequest; i++) {
        avancerMM(cote);
        pivoterGauche(90);
        if ((i % 2) == 1) cote += pas;         // s'élargit tous les 2 côtés
    }
    moteursStop();
    fonctionEnCours = false;
    bleSend(stopRequest ? "LOG:FA1 interrompue" : "DONE:FA1");
}

// ============================================================
//  FA2 — Cercle + rosace circonscrite (>= 4 pétales)
// ============================================================
void faRosace(float rayonCM) {
    fonctionEnCours = true;
    bleSendf("LOG:FA2 rosace r=%.1fcm", rayonCM);
    dessinerCercle(rayonCM);                   // cercle de référence
    int nbPetales = 6;
    for (int p = 0; p < nbPetales && !stopRequest; p++) {
        dessinerCercle(rayonCM / 2.0);         // pétale = cercle moitié
        pivoterGauche(360.0 / nbPetales);      // réparti autour du centre
    }
    moteursStop();
    fonctionEnCours = false;
    bleSend(stopRequest ? "LOG:FA2 interrompue" : "DONE:FA2");
}

// ============================================================
//  FA3 — Rose des vents complète (cercle + 8 directions, N en flèche)
// ============================================================
void faRoseDesVents(float rayonCM) {
    fonctionEnCours = true;
    bleSend("LOG:FA3 rose des vents");
    orienterNord();                            if (stopRequest) { fonctionEnCours=false; return; }
    dessinerCercle(rayonCM);                   // le cercle extérieur
    // 8 rayons depuis le centre, le 1er (Nord) terminé par une pointe pleine
    for (int d = 0; d < 8 && !stopRequest; d++) {
        avancerMM(rayonCM * 10.0);             // rayon vers l'extérieur
        if (d == 0) trianglePlein(25);         // Nord identifiable (flèche)
        avancerMM(-rayonCM * 10.0);            // retour au centre
        pivoterDroite(45.0);                   // direction suivante
    }
    moteursStop();
    fonctionEnCours = false;
    bleSend(stopRequest ? "LOG:FA3 interrompue" : "DONE:FA3");
}

// ============================================================
//  CALIBRATIONS
// ============================================================
void calibrerMag() {
    fonctionEnCours = true;
    bleSend("LOG:CAL_MAG - rotation 360 en cours");
    float minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    angleZ = 0;
    enableMoteurs(true);
    motD(-PWM_PIVOT); motG(PWM_PIVOT);          // pivot lent sur place
    unsigned long tPrec = micros();
    while (fabs(angleZ) < 360.0 - TOLERANCE_GYRO && !stopRequest) {
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0; tPrec = tNow;
        angleZ += (-readFloatGyroZ()) * dt;
        float mx, my;
        Wire.beginTransmission(LIS3MDL_ADDR);
        Wire.write(LIS3MDL_OUT_X_L | 0x80);
        Wire.endTransmission(false);
        Wire.requestFrom(LIS3MDL_ADDR, 6);
        int16_t rx = Wire.read() | (Wire.read() << 8);
        int16_t ry = Wire.read() | (Wire.read() << 8);
        Wire.read(); Wire.read();
        if (rx < minX) minX = rx; if (rx > maxX) maxX = rx;
        if (ry < minY) minY = ry; if (ry > maxY) maxY = ry;
    }
    moteursStop();
    magOffsetX = (maxX + minX) / 2.0;
    magOffsetY = (maxY + minY) / 2.0;
    fonctionEnCours = false;
    bleSendf("DONE:CAL_MAG offsetX=%.0f offsetY=%.0f", magOffsetX, magOffsetY);
}
void calibrerNord() {
    // Le robot regarde physiquement le Nord (vérifié à la boussole) -> on
    // mémorise le cap magnétique courant comme référence "Nord".
    capNordReference = capActuel();
    bleSendf("DONE:CAL_NORD capRef=%.1f", capNordReference);
}

// ============================================================
//  TESTS UNITAIRES (prise en main matériel)
// ============================================================
void envoyerTest(int id, const char* nom, bool pass,
                 const String& mesure, const String& attendu, const String& obs) {
    bleSendf("TEST:%d %s : %s | mesure=%s | attendu=%s | %s",
             id, nom, pass ? "PASS" : "FAIL",
             mesure.c_str(), attendu.c_str(), obs.c_str());
}
void test_sensD() {
    countD = 0; enableMoteurs(true); motD(150); delay(500); moteursStop();
    bool p = countD > 0;
    envoyerTest(0, "Sens roue droite", p, "countD=" + String(countD), "countD>0",
                p ? "OK" : "inverser cablage/encodeur");
}
void test_sensG() {
    countG = 0; enableMoteurs(true); motG(150); delay(500); moteursStop();
    bool p = countG > 0;
    envoyerTest(1, "Sens roue gauche", p, "countG=" + String(countG), "countG>0",
                p ? "OK" : "inverser cablage/encodeur");
}
void test_coupure() {
    countD = 0; enableMoteurs(true); motD(150); delay(300); moteursStop();
    long avant = countD; delay(300); long apres = countD;
    bool p = (apres - avant) < 5;
    envoyerTest(2, "Coupure moteur", p, "delta=" + String(apres - avant), "delta<5",
                p ? "moteur bien coupe" : "inertie/derive");
}
void test_seuilD() {
    int seuil = 0; enableMoteurs(true);
    for (int pwm = 30; pwm <= 255 && !stopRequest; pwm += 5) {
        countD = 0; motD(pwm); delay(200);
        if (countD > 2) { seuil = pwm; break; }
    }
    moteursStop();
    bool p = seuil > 0 && seuil < 200;
    envoyerTest(3, "Seuil demarrage D", p, "PWM=" + String(seuil), "0<seuil<200",
                "seuil=" + String(seuil));
}
void test_seuilG() {
    int seuil = 0; enableMoteurs(true);
    for (int pwm = 30; pwm <= 255 && !stopRequest; pwm += 5) {
        countG = 0; motG(pwm); delay(200);
        if (countG > 2) { seuil = pwm; break; }
    }
    moteursStop();
    bool p = seuil > 0 && seuil < 200;
    envoyerTest(4, "Seuil demarrage G", p, "PWM=" + String(seuil), "0<seuil<200",
                "seuil=" + String(seuil));
}
void test_encodeurs() {
    countG = 0; countD = 0; enableMoteurs(true);
    motD(150); motG(150); delay(1000); moteursStop();
    bool p = countG > 50 && countD > 50;
    envoyerTest(5, "Encodeurs", p,
                "G=" + String(countG) + " D=" + String(countD), "G>50 et D>50",
                p ? "les deux repondent" : "un encodeur muet");
}
void test_cransTour() {
    countG = 0; delay(5000);                    // tourner la roue d'1 tour à la main
    long c = labs(countG);
    bool p = c > 300 && c < 420;
    envoyerTest(6, "Crans par tour", p, "crans=" + String(c), "~" + String(CRANS_PAR_TOUR),
                p ? "calibration coherente" : "ajuster CRANS_PAR_TOUR");
}
void test_gyro() {
    float gMax = 0; unsigned long t = millis();
    while (millis() - t < 3000 && !stopRequest) {
        float g = fabs(readFloatGyroZ());
        if (g > gMax) gMax = g;
        delay(10);
    }
    bool p = gMax > 5.0;
    envoyerTest(7, "Gyroscope actif", p, "gMax=" + String(gMax, 1) + " deg/s",
                ">5 deg/s", p ? "OK (bouger le robot)" : "pas de reponse");
}
void test_deriveGyro() {
    angleZ = 0; unsigned long t = millis();
    while (millis() - t < 10000 && !stopRequest) {
        angleZ += (-readFloatGyroZ()) * 0.01; delay(10);
    }
    bool p = fabs(angleZ) < 2.0;
    envoyerTest(8, "Derive gyro 10s", p, "derive=" + String(angleZ, 2) + " deg",
                "<2 deg", p ? "acceptable" : "recalibrer/au repos");
}
void test_trajectoire() {
    angleZ = 0; countG = 0; countD = 0;
    long cible = mmEnCrans(1000.0);
    enableMoteurs(true);
    unsigned long tPrec = micros();
    while (countG < cible && countD < cible && !stopRequest) {
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0; tPrec = tNow;
        angleZ += (-readFloatGyroZ()) * dt;
        int corr = (int)(angleZ * kp);
        motD(constrain(PWM_AVANCE - corr, 0, 255));
        motG(constrain(PWM_AVANCE + corr, 0, 255));
        delay(10);
    }
    moteursStop();
    bool p = fabs(angleZ) < 5.0;
    envoyerTest(9, "Trajectoire droite 1m", p, "angleZ=" + String(angleZ, 1) + " deg",
                "<5 deg", p ? "OK" : "derive trop forte");
}
void test_pivot90() {
    float a[3] = {0, 0, 0}, moy = 0;
    for (int e = 0; e < 3 && !stopRequest; e++) {
        pivoterDroite(90.0); a[e] = fabs(angleZ); moy += a[e]; delay(1500);
    }
    moy /= 3.0;
    float ecart = 0;
    for (int i = 0; i < 3; i++) ecart = max(ecart, (float)fabs(a[i] - moy));
    bool p = ecart < 3.0;
    envoyerTest(10, "Pivot 90 reproductible", p,
                "ecartMax=" + String(ecart, 1) + " (" + String(a[0],0) + "/" +
                String(a[1],0) + "/" + String(a[2],0) + ")",
                "ecart<3 deg", p ? "reproductible" : "variable");
}
void test_cercleRetour() {
    angleZ = 0; dessinerCercle(20.0);
    float tot = fabs(angleZ);
    bool p = tot > 350.0 && tot < 370.0;
    envoyerTest(11, "Cercle 20cm fermeture", p, "angle=" + String(tot, 0) + " deg",
                "350<angle<370", p ? "cercle complet" : "incomplet/depasse");
}

// ============================================================
//  INTERPRÉTEUR DE COMMANDES (équivalent des routes HTTP)
// ============================================================
void executerCommande(const String &cmdIn) {
    String cmd = cmdIn; cmd.trim();
    bleSendf("LOG:CMD=%s", cmd.c_str());

    // --- utilitaires ---
    if      (cmd == "PING")    { bleSend("PONG"); return; }
    else if (cmd == "STATUS")  {
        bleSendf("STATUS:cap=%.1f,countG=%ld,countD=%ld,angleZ=%.2f,rayon=%.0f,capNord=%.1f,sync=%.2f,pivotCal=%.2f,distCal=%.2f,vA=%d,vP=%d",
                 capActuel(), countG, countD, angleZ, rayonCercle, capNordReference, kpSync, pivotCal, distCal, pwmAvanceSeq, pwmPivotSeq);
        return;
    }
    else if (cmd == "RESET_ODO") { resetPose(); bleSend("DONE:RESET_ODO (0,0,0)"); return; }
    else if (cmd.startsWith("LIGNE:")) {           // reglage live : LIGNE:cap,cross,fwd
        String a = cmd.substring(6);
        int c1 = a.indexOf(','), c2 = a.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > 0) {
            kpLigneCap   = a.substring(0, c1).toFloat();
            kpLigneCross = a.substring(c1 + 1, c2).toFloat();
            suiviFwdMin  = a.substring(c2 + 1).toFloat();
            bleSendf("LOG:LIGNE cap=%.1f cross=%.2f fwd=%.2f", kpLigneCap, kpLigneCross, suiviFwdMin);
        } else bleSend("ERR:format LIGNE:cap,cross,fwd");
        return;
    }
    else if (cmd.startsWith("SYNC:")) {
        kpSync = cmd.substring(5).toFloat();
        if (kpSync < 0) kpSync = 0;
        bleSendf("LOG:SYNC kpSync=%.2f", kpSync);
        return;
    }
    else if (cmd.startsWith("PIVOTCAL:")) {
        pivotCal = cmd.substring(9).toFloat();
        if (pivotCal < 0.50) pivotCal = 0.50;
        if (pivotCal > 2.50) pivotCal = 2.50;
        bleSendf("LOG:PIVOTCAL=%.2f", pivotCal);
        return;
    }
    else if (cmd.startsWith("DISTCAL:")) {
        distCal = cmd.substring(8).toFloat();
        if (distCal < 0.50) distCal = 0.50;
        if (distCal > 3.00) distCal = 3.00;
        bleSendf("LOG:DISTCAL=%.3f", distCal);
        return;
    }
    else if (cmd.startsWith("CALFWD:")) {
        // Format : CALFWD:commande_cm:mesure_cm  ex : CALFWD:20:12
        int sep = cmd.indexOf(':', 7);
        if (sep > 0) {
            float demande = cmd.substring(7, sep).toFloat();
            float mesure  = cmd.substring(sep + 1).toFloat();
            if (demande > 0 && mesure > 0) {
                distCal *= (demande / mesure);
                if (distCal < 0.50) distCal = 0.50;
                if (distCal > 3.00) distCal = 3.00;
                bleSendf("LOG:CALFWD demande=%.1f mesure=%.1f -> DISTCAL=%.3f", demande, mesure, distCal);
            } else {
                bleSend("ERR:format CALFWD:commande_cm:mesure_cm");
            }
        } else {
            bleSend("ERR:format CALFWD:commande_cm:mesure_cm");
        }
        return;
    }
    else if (cmd.startsWith("VAVANCE:")) {
        pwmAvanceSeq = constrain(cmd.substring(8).toInt(), 55, 180);
        bleSendf("LOG:VAVANCE=%d", pwmAvanceSeq);
        return;
    }
    else if (cmd.startsWith("VPIVOT:")) {
        pwmPivotSeq = constrain(cmd.substring(7).toInt(), 55, 170);
        bleSendf("LOG:VPIVOT=%d", pwmPivotSeq);
        return;
    }
    else if (cmd == "LED_ON"  || cmd == "ALLUMER")  { digitalWrite(LED_BUILTIN, HIGH); bleSend("LOG:LED ON");  return; }
    else if (cmd == "LED_OFF" || cmd == "ETEINDRE") { digitalWrite(LED_BUILTIN, LOW);  bleSend("LOG:LED OFF"); return; }

    // --- primitives ---
    else if (cmd == "ROULER") {
        enableMoteurs(true); motD(90); motG(90);
        bleSend("LOG:Rouler (PWM 90)"); return;
    }
    else if (cmd == "AVANCER_DROIT" || cmd == "AVANCE_DROIT") {
        angleZ = 0; enMarche = true; enableMoteurs(true);
        bleSend("LOG:Avance libre asservie en cap"); return;
    }
    else if (cmd.startsWith("FWD:")) {
        float mm = cmd.substring(4).toFloat();
        bleSendf("LOG:FWD %.0f mm", mm); avancerMM(mm); bleSend("DONE:FWD"); return;
    }
    else if (cmd.startsWith("TURNL:")) {
        float a = cmd.substring(6).toFloat();
        pivoterGauche(a); bleSend("DONE:TURNL"); return;
    }
    else if (cmd.startsWith("TURNR:")) {
        float a = cmd.substring(6).toFloat();
        pivoterDroite(a); bleSend("DONE:TURNR"); return;
    }
    else if (cmd.startsWith("ASSERV:")) {
        float mm = cmd.substring(7).toFloat();
        bleSendf("LOG:Asservissement position %.0f mm", mm);
        asservPosition(mm); bleSend("DONE:ASSERV"); return;
    }

    // --- séquences principales ---
    else if (cmd == "SEQ1" || cmd == "ESCALIER" || cmd == "ANGLE_DROIT") {
        sequenceEscalier(); return;
    }
    else if (cmd.startsWith("SEQ2:") || cmd.startsWith("CERCLE:")) {
        // SEQ2:<rayon_cm>   ou (compat) CERCLE:R<rayon_mm>
        float r;
        if (cmd.startsWith("CERCLE:R")) r = cmd.substring(8).toFloat() / 10.0;
        else                            r = cmd.substring(5).toFloat();
        if (r < 2.0 || r > 20.0) { bleSend("ERR:Rayon hors plage (2..20 cm)"); return; }
        bleSendf("LOG:SEQ2 cercle r=%.1f cm", r);
        dessinerCercle(r);
        bleSend(stopRequest ? "LOG:SEQ2 interrompue" : "DONE:SEQ2");
        return;
    }
    else if (cmd == "SEQ3" || cmd == "FLECHE_NORD") { sequenceNord(); return; }

    // --- fonctionnalités avancées ---
    else if (cmd.startsWith("FA1:")) {
        // FA1:<L_cm>:<n>
        int sep = cmd.indexOf(':', 4);
        float L = cmd.substring(4, sep > 0 ? sep : cmd.length()).toFloat();
        int   n = sep > 0 ? cmd.substring(sep + 1).toInt() : 3;
        faCarresSpirale(L, n); return;
    }
    else if (cmd.startsWith("FA2:")) { faRosace(cmd.substring(4).toFloat()); return; }
    else if (cmd.startsWith("FA3")) {
        float r = (cmd.indexOf(':') > 0) ? cmd.substring(cmd.indexOf(':') + 1).toFloat() : 8.0;
        faRoseDesVents(r); return;
    }

    // --- calibrations ---
    else if (cmd == "CAL_MAG"  || cmd == "CALIBRER_MAG") { calibrerMag();  return; }
    else if (cmd == "CAL_NORD")                          { calibrerNord(); return; }

    // --- réglage PID : PID:KP5:KI0.2:KD0.5 ---
    else if (cmd.startsWith("PID:")) {
        int iP = cmd.indexOf("KP"), iI = cmd.indexOf("KI"), iD = cmd.indexOf("KD");
        if (iP >= 0) kp = cmd.substring(iP + 2, cmd.indexOf(':', iP) > 0 ? cmd.indexOf(':', iP) : cmd.length()).toFloat();
        if (iI >= 0) ki = cmd.substring(iI + 2, cmd.indexOf(':', iI) > 0 ? cmd.indexOf(':', iI) : cmd.length()).toFloat();
        if (iD >= 0) kd = cmd.substring(iD + 2).toFloat();
        bleSendf("LOG:PID Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
        return;
    }

    // --- tests unitaires ---
    else if (cmd.startsWith("TEST:")) {
        int id = cmd.substring(5).toInt();
        bleSendf("LOG:Lancement TEST %d", id);
        switch (id) {
            case 0:  test_sensD();        break;
            case 1:  test_sensG();        break;
            case 2:  test_coupure();      break;
            case 3:  test_seuilD();       break;
            case 4:  test_seuilG();       break;
            case 5:  test_encodeurs();    break;
            case 6:  test_cransTour();    break;
            case 7:  test_gyro();         break;
            case 8:  test_deriveGyro();   break;
            case 9:  test_trajectoire();  break;
            case 10: test_pivot90();      break;
            case 11: test_cercleRetour(); break;
            default: bleSendf("ERR:TEST %d inconnu", id); return;
        }
        bleSendf("DONE:TEST%d", id);
        return;
    }

    // --- pilotage manuel : JOY:LY..:L2..:R2.. ---
    else if (cmd.startsWith("JOY:")) {
        int iY = cmd.indexOf("LY"), i2 = cmd.indexOf("L2"), iR = cmd.indexOf("R2");
        if (iY >= 0 && i2 >= 0 && iR >= 0) {
            jLY = cmd.substring(iY + 2, cmd.indexOf(':', iY)).toFloat();
            jL2 = cmd.substring(i2 + 2, cmd.indexOf(':', i2)).toFloat();
            jR2 = cmd.substring(iR + 2).toFloat();
            if (!enMarche) {
                enableMoteurs(true);
                motD((int)(jR2 * 255 - jLY * 128 * jR2) - (int)(jL2 * 255));
                motG((int)(jR2 * 255 + jLY * 128 * jR2) - (int)(jL2 * 255));
            }
        }
        return;
    }

    bleSendf("ERR:UNKNOWN=%s", cmd.c_str());
}

// ============================================================
//  CALLBACKS BLE
// ============================================================
class ServeurCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* s) override {
        bleConnected = true;  digitalWrite(LEDU2, HIGH);
    }
    void onDisconnect(BLEServer* s) override {
        bleConnected = false; digitalWrite(LEDU2, LOW);
        stopRequest = true; stopperImmediat();
        s->startAdvertising();
    }
};
class CmdCallbacks : public BLECharacteristicCallbacks {
    void traiter(String c) {
        c.trim();
        if (c.length() == 0) return;
        if (c == "STOP") {                       // arrêt immédiat (non bloquant)
            stopRequest = true; stopperImmediat();
            bleSend("LOG:STOPPED");
        } else {
            commandePendingStr = c;              // exécutée dans loop()
            commandePending = true;
        }
    }
    void onWrite(BLECharacteristic *pChar) override {
        String data = pChar->getValue().c_str();
        int start = 0, end;
        while ((end = data.indexOf('\n', start)) >= 0) {
            traiter(data.substring(start, end));
            start = end + 1;
        }
        if (start < (int)data.length()) traiter(data.substring(start));
    }
};

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n[BOOT] Drawbot BLE - ECE");

    pinMode(LEDU1, OUTPUT); pinMode(LEDU2, OUTPUT); pinMode(LED_BUILTIN, OUTPUT);
    pinMode(IN_1_D, OUTPUT); pinMode(IN_2_D, OUTPUT); pinMode(EN_D, OUTPUT);
    pinMode(IN_1_G, OUTPUT); pinMode(IN_2_G, OUTPUT); pinMode(EN_G, OUTPUT);
    pinMode(ENC_G_CH_A, INPUT_PULLUP); pinMode(ENC_G_CH_B, INPUT_PULLUP);
    pinMode(ENC_D_CH_A, INPUT_PULLUP); pinMode(ENC_D_CH_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), isrG_A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), isrD_A, RISING);

    Wire.begin(SDA, SCL);
    Wire.setClock(400000);
    initIMU();
    initMag();
    moteursStop();

    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(247);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServeurCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCmdChar = pService->createCharacteristic(
        CMD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pCmdChar->setCallbacks(new CmdCallbacks());
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
//  LOOP
// ============================================================
void loop() {
    // 1) executer une commande BLE en attente (bloquant : OK, mono-tache)
    if (commandePending) {
        commandePending = false;
        stopRequest = false;                  // rearme avant chaque commande
        executerCommande(commandePendingStr);
    }

    // 2) integration du cap au repos + mode "avance libre" asservie
    if (!fonctionEnCours) {
        static unsigned long tPrec = micros();
        unsigned long tNow = micros();
        float dt = (tNow - tPrec) / 1000000.0; tPrec = tNow;
        angleZ += (-readFloatGyroZ()) * dt;
        majOdometrie();                       // pose cartesienne continue

        if (enMarche) {
            int corr = (int)(angleZ * kp);    // correction P sur le cap
            motD(constrain((int)(128 - corr), 0, 255));
            motG(constrain((int)(128 + corr), 0, 255));
        }
    }

    // 3) telemetrie periodique
    envoyerTelemetrie();
}
