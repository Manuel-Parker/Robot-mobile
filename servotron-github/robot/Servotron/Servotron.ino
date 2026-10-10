// =============================================================
// SERVOTRON — programme du robot réel (ESP32, framework Arduino)
//
// Le robot crée son propre réseau Wi-Fi et sert l'IHM "Contrôle du Robot Mobile".
//   1. Se connecter au Wi-Fi  : SERVOTRON   (mot de passe : servotron)
//   2. Ouvrir dans le navigateur : http://192.168.4.1
//
// Le robot exécute uniquement les commandes de l'IHM :
//   avancer / reculer / gauche / droite (tant que le bouton est maintenu), stop,
//   vitesse, formes (cercle, carré, triangle), suivi de ligne, évitement d'obstacle.
//
// Adapté de la simulation Webots. Différence importante : le robot réel n'a pas
// d'encodeurs, les formes sont donc réalisées en temporisation (à calibrer, voir
// la section CALIBRATION).
// =============================================================

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

// =============================================================
// CÂBLAGE — à vérifier avec ton montage
// (repris de tes programmes Cercle / detection_obstacles / suivi_ligne)
// =============================================================
const int PIN_LEFT_PWM = 13;   // EN1
const int PIN_LEFT_DIR = 12;   // DIR1  (GPIO 12 : ne pas le forcer à l'état haut au démarrage)
const int PIN_RIGHT_PWM = 14;  // EN2
const int PIN_RIGHT_DIR = 27;  // DIR2

const int PIN_LINE_LEFT = 22;
const int PIN_LINE_CENTER = 21;
const int PIN_LINE_RIGHT = 19;

const int PIN_TRIG = 5;
const int PIN_ECHO = 4;

// Capteurs IR d'obstacle (optionnels) : -1 = non câblé
const int PIN_IR_OBST_LEFT = -1;
const int PIN_IR_OBST_RIGHT = -1;

// Sens des moteurs : passer à true si une roue tourne à l'envers
const bool INVERT_LEFT = false;
const bool INVERT_RIGHT = false;

// Capteurs de ligne : LOW quand la ligne noire est détectée (comme dans suivi_ligne.ino)
const bool LINE_ACTIVE_LOW = true;
// Capteurs IR d'obstacle de type FC-51 : LOW quand un obstacle est détecté
const bool OBSTACLE_ACTIVE_LOW = true;

// =============================================================
// CALIBRATION — à ajuster sur le vrai robot
// Les formes sont jouées à vitesse fixe (SHAPE_PWM) pour que les durées restent valables.
// =============================================================
const int SHAPE_PWM = 128;             // vitesse des formes (identique à tes essais)
const float MS_PER_CM = 100.0f;        // 5000 ms pour 50 cm (Cercle.ino)
const float MS_PER_DEGREE = 11.1f;     // 1000 ms pour ~90° (Cercle.ino, fonction Gauche)
const unsigned long PAUSE_MS = 300;    // arrêt entre deux segments (limite l'inertie)

const float SQUARE_SIDE_CM = 50.0f;
const float TRIANGLE_SIDE_CM = 50.0f;

const int CIRCLE_PWM_OUTER = 220;      // roue extérieure (Cercle.ino)
const int CIRCLE_PWM_INNER = 85;       // roue intérieure (Cercle.ino)
const unsigned long CIRCLE_MS = 4500;  // durée d'un tour complet (Cercle.ino)

const int PWM_MIN = 90;                // en dessous, les moteurs TT ne démarrent pas
const int OBSTACLE_CM = 25;            // distance de déclenchement de l'évitement
const int CLEAR_CM = 45;               // distance à retrouver avant de repartir

// Régulation PD du suivi de ligne (en unités PWM)
const float LINE_KP = 90.0f;
const float LINE_KD = 60.0f;

// =============================================================
// Réseau
// =============================================================
const char *WIFI_SSID = "SERVOTRON";
const char *WIFI_PASSWORD = "servotron";  // 8 caractères minimum
WebServer server(80);

// =============================================================
// État du robot
// =============================================================
enum class Mode { IDLE, MANUAL, SHAPE, LINE, AVOID };
Mode mode = Mode::IDLE;
char manualCmd = 'S';
int speedPercent = 50;

struct Segment {
  enum Type { STRAIGHT, TURN, CIRCLE, PAUSE } type;
  unsigned long durationMs;
};
Segment shape[16];
int shapeLength = 0;
int segIndex = 0;
unsigned long segStart = 0;

float prevLinePos = 0;
int lastLineSide = 1;

enum class AvoidState { FORWARD, BACK, TURN };
AvoidState avoidState = AvoidState::FORWARD;
unsigned long avoidStart = 0;
int avoidDir = 1;
long lastDistanceCm = 999;
unsigned long lastSonarMs = 0;

unsigned long lastControlMs = 0;
const unsigned long CONTROL_PERIOD_MS = 20;

// =============================================================
// Moteurs
// =============================================================
void setMotor(int pwmPin, int dirPin, bool invert, int speed) {
  speed = constrain(speed, -255, 255);
  bool forward = speed >= 0;
  if (invert) forward = !forward;
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

// Vitesses de -255 à 255 ; positif = marche avant
void setWheels(int left, int right) {
  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, INVERT_LEFT, left);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, INVERT_RIGHT, right);
}

int basePwm() {
  if (speedPercent <= 0) return 0;
  return map(speedPercent, 1, 100, PWM_MIN, 255);
}

void stopAll() {
  setWheels(0, 0);
  mode = Mode::IDLE;
  manualCmd = 'S';
}

// =============================================================
// Capteurs
// =============================================================
bool lineDetected(int pin) {
  const int v = digitalRead(pin);
  return LINE_ACTIVE_LOW ? v == LOW : v == HIGH;
}

bool obstacleDetected(int pin) {
  if (pin < 0) return false;
  const int v = digitalRead(pin);
  return OBSTACLE_ACTIVE_LOW ? v == LOW : v == HIGH;
}

// Distance HC-SR04 en cm (999 si rien n'est détecté). Délai max 25 ms pour ne pas bloquer l'IHM.
long readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  const unsigned long duration = pulseIn(PIN_ECHO, HIGH, 25000);
  if (duration == 0) return 999;
  return duration * 0.0343 / 2;
}

// =============================================================
// Formes : suites de segments temporisés
// =============================================================
void addSegment(Segment::Type type, unsigned long ms) {
  if (shapeLength < 16) shape[shapeLength++] = {type, ms};
}

void startShape(const String &name) {
  stopAll();
  shapeLength = 0;
  const unsigned long turn90 = 90 * MS_PER_DEGREE;
  const unsigned long turn120 = 120 * MS_PER_DEGREE;

  if (name == "carre") {
    for (int i = 0; i < 4; ++i) {
      addSegment(Segment::STRAIGHT, SQUARE_SIDE_CM * MS_PER_CM);
      addSegment(Segment::PAUSE, PAUSE_MS);
      addSegment(Segment::TURN, turn90);
      addSegment(Segment::PAUSE, PAUSE_MS);
    }
  } else if (name == "triangle") {
    for (int i = 0; i < 3; ++i) {
      addSegment(Segment::STRAIGHT, TRIANGLE_SIDE_CM * MS_PER_CM);
      addSegment(Segment::PAUSE, PAUSE_MS);
      addSegment(Segment::TURN, turn120);
      addSegment(Segment::PAUSE, PAUSE_MS);
    }
  } else if (name == "cercle") {
    addSegment(Segment::CIRCLE, CIRCLE_MS);
  } else {
    return;
  }

  mode = Mode::SHAPE;
  segIndex = 0;
  segStart = millis();
}

void stepShape() {
  if (segIndex >= shapeLength) {
    stopAll();
    return;
  }
  const Segment &seg = shape[segIndex];
  switch (seg.type) {
    case Segment::STRAIGHT: setWheels(SHAPE_PWM, SHAPE_PWM); break;
    case Segment::TURN: setWheels(-SHAPE_PWM, SHAPE_PWM); break;  // rotation à gauche sur place
    case Segment::CIRCLE: setWheels(CIRCLE_PWM_OUTER, CIRCLE_PWM_INNER); break;
    case Segment::PAUSE: setWheels(0, 0); break;
  }
  if (millis() - segStart >= seg.durationMs) {
    ++segIndex;
    segStart = millis();
  }
}

// =============================================================
// Suivi de ligne : régulation PD sur la position de la ligne
// =============================================================
void stepLine() {
  const bool l = lineDetected(PIN_LINE_LEFT);
  const bool c = lineDetected(PIN_LINE_CENTER);
  const bool r = lineDetected(PIN_LINE_RIGHT);
  const int v = basePwm();

  if (!l && !c && !r) {
    // Ligne perdue : on pivote du côté où elle a été vue en dernier
    setWheels(-0.6 * v * lastLineSide, 0.6 * v * lastLineSide);
    return;
  }

  // Position de la ligne : +1 = à gauche, 0 = au centre, -1 = à droite
  const float pos = ((l ? 1.0f : 0.0f) - (r ? 1.0f : 0.0f)) / (l + c + r);
  if (pos > 0.2f) lastLineSide = 1;
  else if (pos < -0.2f) lastLineSide = -1;

  const float corr = LINE_KP * pos + LINE_KD * (pos - prevLinePos);
  prevLinePos = pos;
  const float speed = v * (1.0f - 0.4f * fabs(pos));
  setWheels(speed - corr, speed + corr);
}

// =============================================================
// Évitement d'obstacle : machine à états (avance -> recul -> rotation)
// =============================================================
void stepAvoid() {
  const unsigned long now = millis();
  if (now - lastSonarMs >= 60) {  // le HC-SR04 a besoin d'environ 60 ms entre deux mesures
    lastDistanceCm = readDistanceCm();
    lastSonarMs = now;
  }
  const bool irLeft = obstacleDetected(PIN_IR_OBST_LEFT);
  const bool irRight = obstacleDetected(PIN_IR_OBST_RIGHT);
  const int v = basePwm();

  switch (avoidState) {
    case AvoidState::FORWARD:
      if (lastDistanceCm < OBSTACLE_CM || irLeft || irRight) {
        avoidState = AvoidState::BACK;
        avoidStart = now;
        if (irLeft != irRight) avoidDir = irLeft ? -1 : 1;  // obstacle à gauche -> on tourne à droite
        else avoidDir = random(2) ? 1 : -1;
      } else {
        setWheels(v, v);
      }
      break;

    case AvoidState::BACK:  // recul bref pour se dégager
      setWheels(-0.7 * v, -0.7 * v);
      if (now - avoidStart > 400) {
        avoidState = AvoidState::TURN;
        avoidStart = now;
      }
      break;

    case AvoidState::TURN:  // rotation jusqu'à retrouver un passage libre
      setWheels(-0.7 * v * avoidDir, 0.7 * v * avoidDir);
      if ((lastDistanceCm > CLEAR_CM && now - avoidStart > 500 && !irLeft && !irRight) || now - avoidStart > 3000)
        avoidState = AvoidState::FORWARD;
      break;
  }
}

// =============================================================
// Pilotage manuel
// =============================================================
void stepManual() {
  const int v = basePwm();
  switch (manualCmd) {
    case 'F': setWheels(v, v); break;
    case 'B': setWheels(-v, -v); break;
    case 'L': setWheels(-0.6 * v, 0.6 * v); break;
    case 'R': setWheels(0.6 * v, -0.6 * v); break;
    default: setWheels(0, 0);
  }
}

// =============================================================
// Commandes de l'IHM
// =============================================================
void handleCommand(const String &cmd) {
  if (cmd == "avancer" || cmd == "reculer" || cmd == "gauche" || cmd == "droite") {
    if (mode != Mode::MANUAL) stopAll();
    mode = Mode::MANUAL;
    manualCmd = cmd == "avancer" ? 'F' : cmd == "reculer" ? 'B' : cmd == "gauche" ? 'L' : 'R';
  } else if (cmd == "stop") {
    stopAll();
  } else if (cmd.startsWith("vitesse:")) {
    speedPercent = constrain(cmd.substring(8).toInt(), 0, 100);
  } else if (cmd.startsWith("forme:")) {
    startShape(cmd.substring(6));
  } else if (cmd == "ligne") {
    stopAll();
    prevLinePos = 0;
    mode = Mode::LINE;
  } else if (cmd == "evitement") {
    stopAll();
    avoidState = AvoidState::FORWARD;
    mode = Mode::AVOID;
  }
}

// =============================================================
// IHM web (apparence identique à "Contrôle du Robot Mobile").
// Icônes en SVG intégrées : le robot n'a pas accès à Internet.
// =============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Contrôle du Robot Mobile</title>
<style>
  body { font-family: Arial, sans-serif; text-align: center; background: linear-gradient(135deg, #1e3c72, #2a5298);
         color: white; margin: 0; padding: 0; min-height: 100vh; user-select: none; -webkit-user-select: none; }
  h1 { margin-top: 20px; }
  .control-panel { margin-top: 50px; display: flex; justify-content: space-between; align-items: center; padding: 0 20px; }
  .left-panel { display: flex; flex-direction: column; align-items: flex-start; gap: 10px; }
  .left-panel button { margin-bottom: 15px; }
  .right-panel { display: flex; flex-direction: column; align-items: flex-end; gap: 20px; }
  .mode-container { display: flex; gap: 10px; }
  .mode-container > button, .mode-container .dropdown { margin-right: 10px; }
  .horizontal-movement { display: flex; gap: 10px; }
  .horizontal-movement button { margin-right: 15px; }
  button { width: 150px; height: 80px; font-size: 20px; border: none; border-radius: 10px; cursor: pointer; transition: 0.2s;
           display: flex; align-items: center; justify-content: center; box-shadow: 0 4px 6px rgba(0,0,0,0.2); touch-action: none; }
  button:active, button.pressed { transform: scale(0.9); }
  button svg { width: 24px; height: 24px; }
  .btn-avancer { background: #4CAF50; color: white; }
  .btn-reculer { background: #f44336; color: white; }
  .btn-droite, .btn-gauche { background: #008CBA; color: white; }
  .btn-stop { background: #555555; color: white; width: 100px; margin-top: 20px; gap: 6px; }
  .btn-stop svg { width: 16px; height: 16px; }
  .btn-mode { background: #ff9800; color: white; padding: 10px 20px; }
  .dropdown { position: relative; }
  .dropdown-content { display: none; position: absolute; background-color: #f1f1f1; min-width: 160px;
                      box-shadow: 0px 8px 16px rgba(0,0,0,0.2); z-index: 1; border-radius: 5px; }
  .dropdown:hover .dropdown-content, .dropdown.open .dropdown-content { display: block; }
  .dropdown-content button { width: 100%; height: auto; background-color: #ff9800; color: white; border-radius: 0;
                             box-shadow: none; padding: 10px; justify-content: flex-start; font-size: 18px; }
  .dropdown-content button:hover { background-color: #f44336; }
  input[type="range"] { width: 80%; margin: 20px 0; }
</style>
</head>
<body>
  <h1>Contrôle du Robot Mobile</h1>
  <div class="control-panel">
    <div class="left-panel">
      <button class="btn-avancer" data-hold="avancer"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"><path d="M12 19V5M5 12l7-7 7 7"/></svg></button>
      <button class="btn-reculer" data-hold="reculer"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"><path d="M12 5v14M19 12l-7 7-7-7"/></svg></button>
    </div>
    <div class="right-panel">
      <div class="mode-container">
        <div class="dropdown" id="formes">
          <button class="btn-mode" id="formesToggle">Formes</button>
          <div class="dropdown-content">
            <button data-cmd="forme:cercle">Cercle</button>
            <button data-cmd="forme:carre">Carré</button>
            <button data-cmd="forme:triangle">Triangle</button>
          </div>
        </div>
        <button class="btn-mode" data-cmd="ligne">Suivi de Ligne</button>
        <button class="btn-mode" data-cmd="evitement">Evitement d'Obstacle</button>
      </div>
      <div class="horizontal-movement">
        <button class="btn-gauche" data-hold="gauche"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"><path d="M19 12H5M12 19l-7-7 7-7"/></svg></button>
        <button class="btn-droite" data-hold="droite"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"><path d="M5 12h14M12 5l7 7-7 7"/></svg></button>
      </div>
      <button class="btn-stop" data-cmd="stop"><svg viewBox="0 0 24 24" fill="currentColor"><rect x="4" y="4" width="16" height="16"/></svg> Stop</button>
    </div>
  </div>
  <div class="speed-control">
    <label for="speed-range">Vitesse</label>
    <input type="range" id="speed-range" min="0" max="100" value="50" step="1">
    <span id="speed-value">50</span>%
  </div>
<script>
  const send = cmd => fetch('/cmd?c=' + encodeURIComponent(cmd)).catch(() => {});

  document.querySelectorAll('[data-hold]').forEach(btn => {
    const start = e => { e.preventDefault(); btn.classList.add('pressed'); send(btn.dataset.hold); };
    const end = () => { if (!btn.classList.contains('pressed')) return; btn.classList.remove('pressed'); send('stop'); };
    btn.addEventListener('pointerdown', start);
    btn.addEventListener('pointerup', end);
    btn.addEventListener('pointerleave', end);
    btn.addEventListener('pointercancel', end);
  });

  const formes = document.getElementById('formes');
  document.querySelectorAll('[data-cmd]').forEach(btn => {
    btn.addEventListener('click', () => { send(btn.dataset.cmd); formes.classList.remove('open'); });
  });
  document.getElementById('formesToggle').addEventListener('click', e => { e.stopPropagation(); formes.classList.toggle('open'); });
  document.addEventListener('click', () => formes.classList.remove('open'));

  const range = document.getElementById('speed-range');
  const label = document.getElementById('speed-value');
  const sendSpeed = () => { label.textContent = range.value; send('vitesse:' + range.value); };
  range.addEventListener('change', sendSpeed);
  range.addEventListener('input', () => { label.textContent = range.value; });
  sendSpeed();
</script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }

void handleCmd() {
  if (server.hasArg("c")) handleCommand(server.arg("c"));
  server.send(200, "text/plain", "ok");
}

// =============================================================
// Programme principal
// =============================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LINE_LEFT, INPUT);
  pinMode(PIN_LINE_CENTER, INPUT);
  pinMode(PIN_LINE_RIGHT, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  if (PIN_IR_OBST_LEFT >= 0) pinMode(PIN_IR_OBST_LEFT, INPUT);
  if (PIN_IR_OBST_RIGHT >= 0) pinMode(PIN_IR_OBST_RIGHT, INPUT);
  stopAll();

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();

  Serial.print("Servotron prêt. Wi-Fi : ");
  Serial.print(WIFI_SSID);
  Serial.print("  ->  http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  const unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;
  lastControlMs = now;

  switch (mode) {
    case Mode::MANUAL: stepManual(); break;
    case Mode::SHAPE: stepShape(); break;
    case Mode::LINE: stepLine(); break;
    case Mode::AVOID: stepAvoid(); break;
    default: setWheels(0, 0);
  }
}
