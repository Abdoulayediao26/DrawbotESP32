#include <Arduino.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_bt.h"
// ======================================================
// PINS
// ======================================================
#define LEDU1 25
#define LEDU2 26

#define EN_D 23
#define EN_G 4

#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16

#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14

// ======================================================
// BLE
// ======================================================
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcd1234-ab12-ab12-ab12-abcdef123456"

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

bool deviceConnected = false;
bool relancerAdvertising = false;

String commandeBLE = "";
bool commandeBLEDisponible = false;

// ======================================================
// PWM
// ======================================================
#define CH_IN1_D 0
#define CH_IN2_D 1
#define CH_IN1_G 2
#define CH_IN2_G 3

// ======================================================
// PARAMETRES ROBOT
// ======================================================
const float RAYON_ROUE = 0.045;
const float ENTRAXE = 0.075;
const int TICKS_PAR_TOUR = 1095;
const float PERIMETRE = 2.0 * PI * RAYON_ROUE;

// Distance centre robot -> bic
const float OFFSET_STYLO = 0.13;

// ======================================================
// MODES
// ======================================================
enum ModeRobot {
  MODE_STOP,
  MODE_ESCALIER,
  MODE_CERCLE,
  MODE_ROSE
};

ModeRobot modeActuel = MODE_STOP;

// ======================================================
// ENCODEURS / ODOMETRIE
// ======================================================
volatile long ticksG = 0;
volatile long ticksD = 0;

long lastTicksG = 0;
long lastTicksD = 0;

double x_robot = -OFFSET_STYLO;
double y_robot = 0.0;
double theta_robot = 0.0;

// ======================================================
// STRUCT POINT
// ======================================================
struct Point {
  float x;
  float y;
};

// ======================================================
// SEQUENCE 1 : ESCALIER
// ======================================================
const Point PARCOURS_ESCALIER[] = {
  {0.20, 0.00},
  {0.20, 0.08},
  {0.40, 0.08},
  {0.40, 0.16},
  {0.60, 0.16}
};

const int NB_POINTS_ESCALIER = sizeof(PARCOURS_ESCALIER) / sizeof(Point);

int pointEscalierActuel = 0;
bool escalierFini = false;
bool objectifEscalierAtteint = false;

unsigned long tempsArrivePointEscalier = 0;
const unsigned long PAUSE_ENTRE_POINTS = 700;

// ======================================================
// SEQUENCE 3 : ROSE DES VENTS
// Coordonnees :
// +Y = Nord
// +X = Est
// Le N est volontairement plus grand et rempli.
// ======================================================
const Point PARCOURS_ROSE[] = {
  // depart centre
  {0.000, 0.000},

  // cercle exterieur approxime, rayon 10 cm
  {0.000, 0.100},
  {0.038, 0.092},
  {0.071, 0.071},
  {0.092, 0.038},
  {0.100, 0.000},
  {0.092, -0.038},
  {0.071, -0.071},
  {0.038, -0.092},
  {0.000, -0.100},
  {-0.038, -0.092},
  {-0.071, -0.071},
  {-0.092, -0.038},
  {-0.100, 0.000},
  {-0.092, 0.038},
  {-0.071, 0.071},
  {-0.038, 0.092},
  {0.000, 0.100},

  // retour centre
  {0.000, 0.000},

  // grande pointe Nord remplie
  {-0.030, 0.025},
  {0.000, 0.120},
  {0.030, 0.025},
  {-0.024, 0.040},
  {0.024, 0.040},
  {-0.018, 0.060},
  {0.018, 0.060},
  {-0.012, 0.080},
  {0.012, 0.080},
  {-0.006, 0.100},
  {0.006, 0.100},
  {0.000, 0.120},

  // retour centre
  {0.000, 0.000},

  // pointes cardinales
  {0.110, 0.000},   // Est
  {0.000, 0.000},
  {0.000, -0.090},  // Sud
  {0.000, 0.000},
  {-0.110, 0.000},  // Ouest
  {0.000, 0.000},

  // pointes intermediaires
  {0.075, 0.075},   // Nord-Est
  {0.000, 0.000},
  {0.075, -0.075},  // Sud-Est
  {0.000, 0.000},
  {-0.075, -0.075}, // Sud-Ouest
  {0.000, 0.000},
  {-0.075, 0.075},  // Nord-Ouest
  {0.000, 0.000}
};

const int NB_POINTS_ROSE = sizeof(PARCOURS_ROSE) / sizeof(Point);

int pointRoseActuel = 0;
bool roseFini = false;
bool objectifRoseAtteint = false;

unsigned long tempsArrivePointRose = 0;

// cap initial du robot par rapport au Nord, en degres.
// 0 = robot deja oriente vers le Nord.
// 90 = robot oriente vers l'Est.
// Pour respecter le Nord reel, mesurer avec une boussole et envoyer A<angle> ou V<angle>.
float capInitialDegRose = 0.0;

// ======================================================
// PARAMETRES PID COMMUNS POUR ESCALIER ET ROSE
// ======================================================
const float VITESSE_MAX = 150;
const float VITESSE_MIN = 55;

const float DISTANCE_TOLERANCE = 0.015;
const float DISTANCE_TOLERANCE_ROSE = 0.012;
const float ANGLE_TOLERANCE = 0.08;

const float KP_DIST = 850.0;
const float KI_DIST = 20.0;
const float KD_DIST = 80.0;

const float KP_ANGLE = 130.0;
const float KI_ANGLE = 5.0;
const float KD_ANGLE = 25.0;

float erreurDist_precedente = 0;
float integrale_dist = 0;

float erreurAngle_precedente = 0;
float integrale_angle = 0;

unsigned long temps_precedent = 0;

// ======================================================
// SEQUENCE 2 : CERCLE
// ======================================================
const int PWM_DEMARRAGE = 95;
const int PWM_ROTATION = 75;
const int PWM_ROTATION_FIN = 55;

const float COEFF_ROTATION_360 = 1.00;
const float POURCENTAGE_RALENTI = 0.85;

const int SENS_ROTATION = 1;
const unsigned long DUREE_DEMARRAGE_MS = 600;

long startTicksG = 0;
long startTicksD = 0;
long cibleTicksRotation = 0;

bool cercleActif = false;
bool cercleFini = false;

unsigned long tempsDebutCercle = 0;
float dernierRayonCommande = 10.0;

// ======================================================
// PROTOTYPES
// ======================================================
void arreterMoteurs();
void setMoteurs(float vitesseGauche, float vitesseDroite);
void traiterCommande(String cmd);
void afficherDebug();

// ======================================================
// INTERRUPTIONS ENCODEURS
// ======================================================
void IRAM_ATTR encoderG_A() {
  bool b = digitalRead(ENC_G_CH_B);
  ticksG += (b ? -1 : 1);
}

void IRAM_ATTR encoderD_A() {
  bool b = digitalRead(ENC_D_CH_B);
  ticksD += (b ? 1 : -1);
}

// ======================================================
// MOTEURS
// ======================================================
void motorsInit() {
  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);

  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);

  ledcSetup(CH_IN1_D, 1000, 8);
  ledcSetup(CH_IN2_D, 1000, 8);
  ledcSetup(CH_IN1_G, 1000, 8);
  ledcSetup(CH_IN2_G, 1000, 8);

  ledcAttachPin(IN_1_D, CH_IN1_D);
  ledcAttachPin(IN_2_D, CH_IN2_D);
  ledcAttachPin(IN_1_G, CH_IN1_G);
  ledcAttachPin(IN_2_G, CH_IN2_G);
}

void setMoteurs(float vitesseGauche, float vitesseDroite) {
  vitesseGauche = constrain(vitesseGauche, -255, 255);
  vitesseDroite = constrain(vitesseDroite, -255, 255);

  // Moteur gauche
  if (vitesseGauche >= 0) {
    ledcWrite(CH_IN1_G, (int)vitesseGauche);
    ledcWrite(CH_IN2_G, 0);
  } else {
    ledcWrite(CH_IN1_G, 0);
    ledcWrite(CH_IN2_G, (int)(-vitesseGauche));
  }

  // Moteur droit
  if (vitesseDroite >= 0) {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, (int)vitesseDroite);
  } else {
    ledcWrite(CH_IN1_D, (int)(-vitesseDroite));
    ledcWrite(CH_IN2_D, 0);
  }
}

void arreterMoteurs() {
  ledcWrite(CH_IN1_D, 0);
  ledcWrite(CH_IN2_D, 0);
  ledcWrite(CH_IN1_G, 0);
  ledcWrite(CH_IN2_G, 0);
}

// ======================================================
// OUTILS
// ======================================================
double normaliserAngle(double angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

void lireTicks(long &g, long &d) {
  noInterrupts();
  g = ticksG;
  d = ticksD;
  interrupts();
}

void resetTicks() {
  noInterrupts();
  ticksG = 0;
  ticksD = 0;
  lastTicksG = 0;
  lastTicksD = 0;
  interrupts();

  startTicksG = 0;
  startTicksD = 0;
}

void getPositionStylo(double &x_stylo, double &y_stylo) {
  x_stylo = x_robot + OFFSET_STYLO * cos(theta_robot);
  y_stylo = y_robot + OFFSET_STYLO * sin(theta_robot);
}

float extraireNombre(String txt) {
  txt.replace(',', '.');

  String nombre = "";

  for (int i = 0; i < txt.length(); i++) {
    char c = txt.charAt(i);

    if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
      nombre += c;
    }
  }

  if (nombre.length() == 0) {
    return NAN;
  }

  return nombre.toFloat();
}

// ======================================================
// STOP GENERAL
// ======================================================
void stopRobot() {
  modeActuel = MODE_STOP;

  cercleActif = false;
  cercleFini = false;

  arreterMoteurs();

  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);

  Serial.println("STOP GENERAL !");
}

// ======================================================
// PID
// ======================================================
float calculerPID_Distance(float erreur, float dt) {
  float P = KP_DIST * erreur;

  integrale_dist += erreur * dt;
  integrale_dist = constrain(integrale_dist, -0.4, 0.4);
  float I = KI_DIST * integrale_dist;

  float derivee = (erreur - erreurDist_precedente) / dt;
  float D = KD_DIST * derivee;

  erreurDist_precedente = erreur;

  return P + I + D;
}

float calculerPID_Angle(float erreur, float dt) {
  erreur = normaliserAngle(erreur);

  float P = KP_ANGLE * erreur;

  integrale_angle += erreur * dt;
  integrale_angle = constrain(integrale_angle, -0.25, 0.25);
  float I = KI_ANGLE * integrale_angle;

  float derivee = normaliserAngle(erreur - erreurAngle_precedente) / dt;
  float D = KD_ANGLE * derivee;

  erreurAngle_precedente = erreur;

  return P + I + D;
}

void resetPID() {
  integrale_dist = 0;
  integrale_angle = 0;
  erreurDist_precedente = 0;
  erreurAngle_precedente = 0;
  temps_precedent = millis();
}

// ======================================================
// ODOMETRIE
// ======================================================
void mettreAJourOdometrie() {
  noInterrupts();
  long currentG = ticksG;
  long currentD = ticksD;
  interrupts();

  long dG = currentG - lastTicksG;
  long dD = currentD - lastTicksD;

  if (dG == 0 && dD == 0) return;

  lastTicksG = currentG;
  lastTicksD = currentD;

  double distG = (dG * PERIMETRE) / TICKS_PAR_TOUR;
  double distD = (dD * PERIMETRE) / TICKS_PAR_TOUR;

  double dDist = (distG + distD) / 2.0;
  double dTheta = (distD - distG) / ENTRAXE;

  if (abs(dTheta) > 0.0001) {
    double R = dDist / dTheta;

    double dx = R * (sin(theta_robot + dTheta) - sin(theta_robot));
    double dy = R * (cos(theta_robot) - cos(theta_robot + dTheta));

    x_robot += dx;
    y_robot += dy;
  } else {
    x_robot += dDist * cos(theta_robot);
    y_robot += dDist * sin(theta_robot);
  }

  theta_robot = normaliserAngle(theta_robot + dTheta);
}

// ======================================================
// NAVIGATION COMMUNE VERS UN POINT
// ======================================================
bool naviguerVersPoint(Point cible, float tolerance, const char* nomSequence, int indexPoint, int nbPoints) {
  mettreAJourOdometrie();

  unsigned long temps_actuel = millis();
  float dt = (temps_actuel - temps_precedent) / 1000.0;

  if (dt <= 0.001) dt = 0.02;
  temps_precedent = temps_actuel;

  double x_stylo, y_stylo;
  getPositionStylo(x_stylo, y_stylo);

  float dx = cible.x - x_stylo;
  float dy = cible.y - y_stylo;

  float distanceRestante = sqrt(dx * dx + dy * dy);

  if (distanceRestante < tolerance) {
    arreterMoteurs();

    Serial.print(nomSequence);
    Serial.print(" | Point atteint : ");
    Serial.print(indexPoint + 1);
    Serial.print("/");
    Serial.print(nbPoints);
    Serial.print(" -> (");
    Serial.print(cible.x * 100);
    Serial.print(" cm, ");
    Serial.print(cible.y * 100);
    Serial.println(" cm)");

    return true;
  }

  digitalWrite(LEDU1, (millis() / 300) % 2);
  digitalWrite(LEDU2, LOW);

  float angleVersCible = atan2(dy, dx);
  float erreurAngle = normaliserAngle(angleVersCible - theta_robot);

  bool doitReculer = false;
  float angleCorrection = erreurAngle;

  if (abs(erreurAngle) > 2.0) {
    float angleRecul = normaliserAngle(angleVersCible + PI);
    float erreurRecul = normaliserAngle(angleRecul - theta_robot);

    if (abs(erreurRecul) < abs(erreurAngle) - 0.5) {
      doitReculer = true;
      angleCorrection = erreurRecul;
    }
  }

  float cmdDist = calculerPID_Distance(distanceRestante, dt);

  float facteurAngle = 1.0;

  if (abs(angleCorrection) > ANGLE_TOLERANCE) {
    facteurAngle = 1.0 - (abs(angleCorrection) / PI);
    facteurAngle = constrain(facteurAngle, 0.25, 1.0);
  }

  float vitesseAvance = cmdDist * facteurAngle;

  if (doitReculer) {
    vitesseAvance = -abs(vitesseAvance);
  }

  vitesseAvance = constrain(vitesseAvance, -VITESSE_MAX, VITESSE_MAX);

  if (abs(vitesseAvance) > 0 && abs(vitesseAvance) < VITESSE_MIN) {
    vitesseAvance = vitesseAvance > 0 ? VITESSE_MIN : -VITESSE_MIN;
  }

  float cmdAngle = calculerPID_Angle(angleCorrection, dt);
  cmdAngle = constrain(cmdAngle, -VITESSE_MAX, VITESSE_MAX);

  float vitesseG = vitesseAvance - cmdAngle;
  float vitesseD = vitesseAvance + cmdAngle;

  setMoteurs(vitesseG, vitesseD);

  static unsigned long dernierDebugNav = 0;

  if (millis() - dernierDebugNav > 600) {
    dernierDebugNav = millis();

    Serial.print(nomSequence);
    Serial.print(" | Point ");
    Serial.print(indexPoint + 1);
    Serial.print("/");
    Serial.print(nbPoints);
    Serial.print(" | Stylo = (");
    Serial.print(x_stylo * 100, 1);
    Serial.print(", ");
    Serial.print(y_stylo * 100, 1);
    Serial.print(") cm | dist = ");
    Serial.print(distanceRestante * 100, 1);
    Serial.print(" cm | angle = ");
    Serial.print(angleCorrection * 180.0 / PI, 1);
    Serial.println(" deg");
  }

  return false;
}

// ======================================================
// SEQUENCE 1 : ESCALIER
// ======================================================
void resetEscalier() {
  resetTicks();

  x_robot = -OFFSET_STYLO;
  y_robot = 0.0;
  theta_robot = 0.0;

  pointEscalierActuel = 0;
  escalierFini = false;
  objectifEscalierAtteint = false;

  tempsArrivePointEscalier = 0;
  resetPID();

  Serial.println("Reset escalier : stylo en (0,0)");
}

void lancerEscalier() {
  stopRobot();
  resetEscalier();

  modeActuel = MODE_ESCALIER;

  digitalWrite(LEDU1, HIGH);
  digitalWrite(LEDU2, LOW);

  Serial.println("====================================");
  Serial.println("ESCALIER LANCE");
  Serial.println("Commande interface : R ou E");
  Serial.println("====================================");
}

void updateEscalier() {
  if (escalierFini) {
    arreterMoteurs();
    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);
    modeActuel = MODE_STOP;
    Serial.println("Escalier fini. Envoie R ou E pour relancer.");
    return;
  }

  if (objectifEscalierAtteint) {
    arreterMoteurs();

    digitalWrite(LEDU1, HIGH);
    digitalWrite(LEDU2, HIGH);

    if (millis() - tempsArrivePointEscalier > PAUSE_ENTRE_POINTS) {
      if (pointEscalierActuel < NB_POINTS_ESCALIER - 1) {
        pointEscalierActuel++;
        objectifEscalierAtteint = false;
        resetPID();

        Serial.print("Escalier | Prochain point : ");
        Serial.print(pointEscalierActuel + 1);
        Serial.print("/");
        Serial.println(NB_POINTS_ESCALIER);
      } else {
        escalierFini = true;
        Serial.println("Escalier termine !");
      }
    }

    return;
  }

  bool atteint = naviguerVersPoint(
    PARCOURS_ESCALIER[pointEscalierActuel],
    DISTANCE_TOLERANCE,
    "Escalier",
    pointEscalierActuel,
    NB_POINTS_ESCALIER
  );

  if (atteint) {
    objectifEscalierAtteint = true;
    tempsArrivePointEscalier = millis();
  }
}

// ======================================================
// SEQUENCE 2 : CERCLE
// ======================================================
long calculerTicksPour360() {
  float distanceRoue = PI * ENTRAXE;

  long ticks = (long)((distanceRoue / PERIMETRE) * TICKS_PAR_TOUR);
  ticks = (long)(ticks * COEFF_ROTATION_360);

  return ticks;
}

void lancerCercle(float rayonCommande) {
  stopRobot();

  dernierRayonCommande = rayonCommande;

  resetTicks();
  lireTicks(startTicksG, startTicksD);

  cibleTicksRotation = calculerTicksPour360();

  cercleActif = true;
  cercleFini = false;
  modeActuel = MODE_CERCLE;

  tempsDebutCercle = millis();

  digitalWrite(LEDU1, HIGH);
  digitalWrite(LEDU2, HIGH);

  Serial.println("====================================");
  Serial.println("CERCLE LANCE");
  Serial.print("Commande interface : C");
  Serial.println(rayonCommande);
  Serial.print("Rayon reel environ avec bic fixe = ");
  Serial.print(OFFSET_STYLO * 100.0);
  Serial.println(" cm");
  Serial.print("Cible ticks 360 = ");
  Serial.println(cibleTicksRotation);
  Serial.println("====================================");
}

void updateCercle() {
  if (!cercleActif) {
    arreterMoteurs();
    return;
  }

  long g, d;
  lireTicks(g, d);

  long progG = abs(g - startTicksG);
  long progD = abs(d - startTicksD);

  long progression = (progG + progD) / 2;

  if (progression >= cibleTicksRotation) {
    arreterMoteurs();

    cercleActif = false;
    cercleFini = true;
    modeActuel = MODE_STOP;

    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);

    Serial.println("Cercle termine !");
    Serial.print("Ticks G = ");
    Serial.print(progG);
    Serial.print(" | Ticks D = ");
    Serial.println(progD);

    return;
  }

  float ratio = (float)progression / (float)cibleTicksRotation;

  int pwm;

  if (millis() - tempsDebutCercle < DUREE_DEMARRAGE_MS) {
    pwm = PWM_DEMARRAGE;
  } else if (ratio > POURCENTAGE_RALENTI) {
    pwm = PWM_ROTATION_FIN;
  } else {
    pwm = PWM_ROTATION;
  }

  int erreurSync = progG - progD;
  int correction = constrain((int)(erreurSync * 0.35), -10, 10);

  int pwmG = pwm - correction;
  int pwmD = pwm + correction;

  pwmG = constrain(pwmG, 45, 110);
  pwmD = constrain(pwmD, 45, 110);

  if (SENS_ROTATION == 1) {
    setMoteurs(-pwmG, pwmD);
  } else {
    setMoteurs(pwmG, -pwmD);
  }

  digitalWrite(LEDU1, (millis() / 300) % 2);

  static unsigned long dernierDebugCercle = 0;

  if (millis() - dernierDebugCercle > 700) {
    dernierDebugCercle = millis();

    Serial.print("Cercle progression = ");
    Serial.print(ratio * 100.0, 1);
    Serial.print("% | G = ");
    Serial.print(progG);
    Serial.print(" | D = ");
    Serial.print(progD);
    Serial.print(" | PWM = ");
    Serial.println(pwm);
  }
}

// ======================================================
// SEQUENCE 3 : ROSE DES VENTS
// ======================================================
void setCapInitialRose(float capDeg) {
  while (capDeg < 0) capDeg += 360.0;
  while (capDeg >= 360.0) capDeg -= 360.0;

  capInitialDegRose = capDeg;

  Serial.print("Cap initial rose memorise = ");
  Serial.print(capInitialDegRose);
  Serial.println(" deg");
}

void resetRose() {
  resetTicks();

  // Conversion cap boussole vers angle mathematique :
  // cap 0 deg = Nord = +Y = PI/2
  // cap 90 deg = Est = +X = 0
  theta_robot = (90.0 - capInitialDegRose) * PI / 180.0;
  theta_robot = normaliserAngle(theta_robot);

  // Le stylo demarre a (0,0), le centre robot est derriere le stylo.
  x_robot = -OFFSET_STYLO * cos(theta_robot);
  y_robot = -OFFSET_STYLO * sin(theta_robot);

  pointRoseActuel = 0;
  roseFini = false;
  objectifRoseAtteint = false;

  tempsArrivePointRose = 0;
  resetPID();

  Serial.println("Reset rose des vents : stylo en (0,0)");
  Serial.print("Theta robot initial = ");
  Serial.print(theta_robot * 180.0 / PI);
  Serial.println(" deg");
}

void lancerRose() {
  stopRobot();
  resetRose();

  modeActuel = MODE_ROSE;

  digitalWrite(LEDU1, HIGH);
  digitalWrite(LEDU2, LOW);

  Serial.println("====================================");
  Serial.println("ROSE DES VENTS LANCEE");
  Serial.print("Cap initial utilise = ");
  Serial.print(capInitialDegRose);
  Serial.println(" deg");
  Serial.println("Le Nord est dessine vers +Y.");
  Serial.println("====================================");
}

void updateRose() {
  if (roseFini) {
    arreterMoteurs();
    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);
    modeActuel = MODE_STOP;
    Serial.println("Rose des vents terminee. Envoie V pour relancer.");
    return;
  }

  if (objectifRoseAtteint) {
    arreterMoteurs();

    digitalWrite(LEDU1, HIGH);
    digitalWrite(LEDU2, HIGH);

    if (millis() - tempsArrivePointRose > 350) {
      if (pointRoseActuel < NB_POINTS_ROSE - 1) {
        pointRoseActuel++;
        objectifRoseAtteint = false;
        resetPID();

        Serial.print("Rose | Prochain point : ");
        Serial.print(pointRoseActuel + 1);
        Serial.print("/");
        Serial.println(NB_POINTS_ROSE);
      } else {
        roseFini = true;
        Serial.println("Rose des vents terminee !");
      }
    }

    return;
  }

  bool atteint = naviguerVersPoint(
    PARCOURS_ROSE[pointRoseActuel],
    DISTANCE_TOLERANCE_ROSE,
    "Rose",
    pointRoseActuel,
    NB_POINTS_ROSE
  );

  if (atteint) {
    objectifRoseAtteint = true;
    tempsArrivePointRose = millis();
  }
}

// ======================================================
// DEBUG
// ======================================================
void afficherDebug() {
  long g, d;
  lireTicks(g, d);

  Serial.println("===== DEBUG GENERAL =====");

  Serial.print("Mode = ");
  if (modeActuel == MODE_STOP) Serial.println("STOP");
  else if (modeActuel == MODE_ESCALIER) Serial.println("ESCALIER");
  else if (modeActuel == MODE_CERCLE) Serial.println("CERCLE");
  else if (modeActuel == MODE_ROSE) Serial.println("ROSE");

  Serial.print("BLE = ");
  Serial.println(deviceConnected ? "CONNECTE" : "DECONNECTE");

  Serial.print("Ticks G = ");
  Serial.print(g);
  Serial.print(" | Ticks D = ");
  Serial.println(d);

  Serial.print("Robot = ");
  Serial.print(x_robot * 100, 1);
  Serial.print(" cm, ");
  Serial.print(y_robot * 100, 1);
  Serial.print(" cm | theta = ");
  Serial.print(theta_robot * 180.0 / PI, 1);
  Serial.println(" deg");

  double x_stylo, y_stylo;
  getPositionStylo(x_stylo, y_stylo);

  Serial.print("Stylo = ");
  Serial.print(x_stylo * 100, 1);
  Serial.print(" cm, ");
  Serial.print(y_stylo * 100, 1);
  Serial.println(" cm");

  Serial.print("Rayon cercle bic fixe = ");
  Serial.print(OFFSET_STYLO * 100.0);
  Serial.println(" cm");

  Serial.print("Cap initial rose = ");
  Serial.print(capInitialDegRose);
  Serial.println(" deg");

  if (modeActuel == MODE_ESCALIER) {
    Serial.print("Point escalier = ");
    Serial.print(pointEscalierActuel + 1);
    Serial.print("/");
    Serial.println(NB_POINTS_ESCALIER);
  }

  if (modeActuel == MODE_CERCLE) {
    Serial.print("Cible ticks 360 = ");
    Serial.println(cibleTicksRotation);

    Serial.print("Dernier rayon commande = ");
    Serial.print(dernierRayonCommande);
    Serial.println(" cm");
  }

  if (modeActuel == MODE_ROSE) {
    Serial.print("Point rose = ");
    Serial.print(pointRoseActuel + 1);
    Serial.print("/");
    Serial.println(NB_POINTS_ROSE);
  }
}

// ======================================================
// COMMANDES
// ======================================================
void traiterCommande(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  Serial.print("[CMD] Recu : ");
  Serial.println(cmd);

  digitalWrite(LEDU2, HIGH);

  if (cmd == "S") {
    stopRobot();
    return;
  }

  if (cmd == "D") {
    afficherDebug();
    digitalWrite(LEDU2, LOW);
    return;
  }

  // A<angle> : memorise le cap initial pour la rose.
  // Exemple : A0 si le robot est deja oriente Nord.
  // Exemple : A90 si le robot est oriente Est.
  if (cmd.startsWith("A")) {
    float cap = extraireNombre(cmd);
    if (isnan(cap)) cap = 0.0;
    setCapInitialRose(cap);
    digitalWrite(LEDU2, LOW);
    return;
  }

  // Rose des vents :
  // V = lance avec le dernier cap memorise
  // V45 = memorise 45 deg puis lance
  if (cmd == "V" || cmd == "ROSE" || cmd == "ROSEDESVENTS" || cmd == "N" || cmd == "NORD" || cmd.startsWith("V")) {
    if (cmd.startsWith("V") && cmd.length() > 1) {
      float cap = extraireNombre(cmd);
      if (!isnan(cap)) {
        setCapInitialRose(cap);
      }
    }

    lancerRose();
    return;
  }

  // Escalier
  if (cmd == "R" || cmd == "E" || cmd == "ESC" || cmd == "ESCALIER") {
    lancerEscalier();
    return;
  }

  // Cercle : C2, C5, C10, C20
  if (cmd.startsWith("C")) {
    float rayon = extraireNombre(cmd);

    if (isnan(rayon)) {
      rayon = 10.0;
    }

    lancerCercle(rayon);
    return;
  }

  digitalWrite(LEDU2, LOW);

  Serial.println("Commande inconnue.");
  Serial.println("Utilise : R/E = escalier | C2/C5/C10/C20 = cercle | V/Vangle = rose | Aangle = cap | S = stop | D = debug");
}

// ======================================================
// BLE CALLBACKS STABLES
// ======================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    relancerAdvertising = false;
    digitalWrite(LEDU1, HIGH);
    Serial.println("[BLE] Connecte");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    relancerAdvertising = true;
    digitalWrite(LEDU1, LOW);
    Serial.println("[BLE] Deconnecte - relance advertising demandee");
  }
};

class CharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    String val = String(pChar->getValue().c_str());
    val.trim();

    commandeBLE = val;
    commandeBLEDisponible = true;
  }
};

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  pinMode(LEDU1, OUTPUT);
  pinMode(LEDU2, OUTPUT);

  digitalWrite(LEDU1, LOW);
  digitalWrite(LEDU2, LOW);

  pinMode(ENC_G_CH_A, INPUT_PULLUP);
  pinMode(ENC_G_CH_B, INPUT_PULLUP);
  pinMode(ENC_D_CH_A, INPUT_PULLUP);
  pinMode(ENC_D_CH_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), encoderG_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), encoderD_A, RISING);

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  delay(500);

  motorsInit();
  arreterMoteurs();

  for (int i = 0; i < 3; i++) {
    digitalWrite(LEDU1, HIGH);
    delay(150);
    digitalWrite(LEDU1, LOW);
    delay(150);
  }

  BLEDevice::init("Drawbot_Escalier");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  pCharacteristic->setCallbacks(new CharCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  cibleTicksRotation = calculerTicksPour360();
  temps_precedent = millis();

  Serial.println("====================================");
  Serial.println("Drawbot pret !");
  Serial.println("Nom BLE : Drawbot_Escalier");
  Serial.println("Commandes :");
  Serial.println("R ou E = escalier");
  Serial.println("C2, C5, C10, C20 = cercle");
  Serial.println("A<angle> = cap initial rose");
  Serial.println("V ou V<angle> = rose des vents");
  Serial.println("S = stop");
  Serial.println("D = debug");
  Serial.println("====================================");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  // Relance BLE faite dans le loop, pas dans le callback
  if (relancerAdvertising && !deviceConnected) {
    delay(200);
    BLEDevice::startAdvertising();
    relancerAdvertising = false;
    Serial.println("[BLE] Advertising relance");
  }

  // Commande reçue par BLE
  if (commandeBLEDisponible) {
    String cmd = commandeBLE;
    commandeBLEDisponible = false;
    commandeBLE = "";
    traiterCommande(cmd);
  }

  // Commande reçue par le moniteur série
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    traiterCommande(cmd);
  }

  if (modeActuel == MODE_ESCALIER) {
    updateEscalier();
  } 
  else if (modeActuel == MODE_CERCLE) {
    updateCercle();
  } 
  else if (modeActuel == MODE_ROSE) {
    updateRose();
  }
  else {
    arreterMoteurs();

    if (!deviceConnected) {
      digitalWrite(LEDU1, (millis() / 1000) % 2);
    }
  }

  // LED2 retombe après réception commande
  static unsigned long tLed2 = 0;
  if (digitalRead(LEDU2) == HIGH && tLed2 == 0) {
    tLed2 = millis();
  }
  if (tLed2 != 0 && millis() - tLed2 > 120) {
    digitalWrite(LEDU2, LOW);
    tLed2 = 0;
  }

  delay(20);
}
