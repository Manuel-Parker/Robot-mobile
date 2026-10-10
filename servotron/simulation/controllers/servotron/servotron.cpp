// =============================================================
// Servotron — contrôleur Webots
// Le robot exécute uniquement les commandes de son IHM (servotron_window) :
//   avancer / reculer / gauche / droite / stop, vitesse,
//   formes (cercle, carré, triangle), suivi de ligne, évitement d'obstacles.
// =============================================================

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Pen.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace webots;

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double WHEEL_RADIUS = 0.035;  // m
// Écartement effectif des roues, calibré : l'entraxe géométrique vaut 0,304 m mais les pneus
// portent sur leur bord extérieur. Mesuré en simulation sur un tour complet (360° commandés -> 336° réels).
constexpr double AXLE_LENGTH = 0.326;  // m
constexpr double MAX_WHEEL_SPEED = 14.0;  // rad/s

enum class DriveMode { IDLE, MANUAL, SHAPE, LINE, AVOID };

struct Segment {
  enum Type { STRAIGHT, TURN, ARC } type;
  double value;   // distance (m) ou angle (rad)
  double radius;  // rayon de l'arc (m)
};

struct Pose2D {
  double x, y, yaw;
};

double wrapAngle(double a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

// Positions de départ des démonstrations (repère du monde)
const Pose2D START_SQUARE = {-2.1, 0.3, 0.0};
const Pose2D START_CIRCLE = {-1.0, 0.3, 0.0};
const Pose2D START_TRIANGLE = {-1.95, 1.4, 0.0};
const Pose2D START_LINE = {0.45, 0.0, PI / 2};
const Pose2D START_AVOID = {-1.35, -1.1, 0.0};

class Servotron : public Supervisor {
public:
  Servotron();
  void run();

private:
  // ---- matériel ----
  int mTimeStep;
  Motor *mLeftMotor, *mRightMotor;
  PositionSensor *mLeftEncoder, *mRightEncoder;
  DistanceSensor *mSonar, *mIrLeft, *mIrRight;
  DistanceSensor *mGround[3];  // gauche, centre, droite
  Pen *mPen;

  // ---- état ----
  DriveMode mMode = DriveMode::IDLE;
  char mManualCmd = 'S';
  double mSpeedPercent = 50;

  // odométrie
  double mX = 0, mY = 0, mTheta = 0;
  double mPrevLeft = 0, mPrevRight = 0;
  double mSegDistance = 0, mSegTurned = 0;

  // formes
  std::vector<Segment> mShape;
  size_t mSegIndex = 0;
  double mHeadingRef = 0;
  double mPrevHeadingError = 0;
  int mSettleSteps = 0;
  std::string mShapeName;
  Pose2D mShapeStart{};

  // suivi de ligne
  double mGroundWhite = 660, mGroundBlack = 130;  // mesurés en simulation
  double mPrevLinePos = 0, mLastLineSide = 1;

  // évitement
  bool mAvoidTurning = false;
  double mAvoidDir = 1, mAvoidTimer = 0;

  // ---- fonctions ----
  double baseSpeed() const { return 2.0 + 8.0 * mSpeedPercent / 100.0; }
  void setWheels(double left, double right);
  void updateOdometry();
  void resetOdometry(const Pose2D &p);
  void teleport(const Pose2D &p);
  void stopAll();

  void handleCommand(const std::string &cmd);
  void startShape(const std::string &name);
  void startLine();
  void startAvoid();

  void stepManual();
  void stepShape();
  void stepLine();
  void stepAvoid();

  double groundDarkness(int i) const;

  // ---- mode test de développement (variable d'environnement SERVOTRON_TEST) ----
  bool mTest = false;
  void testStep(double t);
};

Servotron::Servotron() {
  mTimeStep = static_cast<int>(getBasicTimeStep());

  mLeftMotor = getMotor("left wheel motor");
  mRightMotor = getMotor("right wheel motor");
  mLeftMotor->setPosition(INFINITY);
  mRightMotor->setPosition(INFINITY);
  setWheels(0, 0);

  mLeftEncoder = getPositionSensor("left wheel sensor");
  mRightEncoder = getPositionSensor("right wheel sensor");
  mLeftEncoder->enable(mTimeStep);
  mRightEncoder->enable(mTimeStep);

  mSonar = getDistanceSensor("sonar");
  mIrLeft = getDistanceSensor("ir left");
  mIrRight = getDistanceSensor("ir right");
  mGround[0] = getDistanceSensor("gs left");
  mGround[1] = getDistanceSensor("gs center");
  mGround[2] = getDistanceSensor("gs right");
  for (DistanceSensor *s : {mSonar, mIrLeft, mIrRight, mGround[0], mGround[1], mGround[2]})
    s->enable(mTimeStep);

  mPen = getPen("pen");
  mPen->write(false);

  mTest = std::getenv("SERVOTRON_TEST") != nullptr;
}

void Servotron::setWheels(double left, double right) {
  mLeftMotor->setVelocity(clamp(left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED));
  mRightMotor->setVelocity(clamp(right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED));
}

void Servotron::updateOdometry() {
  const double l = mLeftEncoder->getValue();
  const double r = mRightEncoder->getValue();
  if (std::isnan(l) || std::isnan(r)) return;

  const double dl = (l - mPrevLeft) * WHEEL_RADIUS;
  const double dr = (r - mPrevRight) * WHEEL_RADIUS;
  mPrevLeft = l;
  mPrevRight = r;

  const double dc = (dl + dr) / 2.0;
  const double dtheta = (dr - dl) / AXLE_LENGTH;
  mX += dc * std::cos(mTheta + dtheta / 2);
  mY += dc * std::sin(mTheta + dtheta / 2);
  mTheta = wrapAngle(mTheta + dtheta);
  mSegDistance += dc;
  mSegTurned += dtheta;
}

void Servotron::resetOdometry(const Pose2D &p) {
  mX = p.x;
  mY = p.y;
  mTheta = p.yaw;
  mPrevLeft = mLeftEncoder->getValue();
  mPrevRight = mRightEncoder->getValue();
  mSegDistance = mSegTurned = 0;
}

// Replace le robot (le contrôleur est superviseur) pour lancer une démo au bon endroit
void Servotron::teleport(const Pose2D &p) {
  setWheels(0, 0);
  mPen->write(false);
  Node *self = getSelf();
  const double translation[3] = {p.x, p.y, 0.0};
  const double rotation[4] = {0, 0, 1, p.yaw};
  self->getField("translation")->setSFVec3f(translation);
  self->getField("rotation")->setSFRotation(rotation);
  self->resetPhysics();
  step(mTimeStep);
  resetOdometry(p);
}

void Servotron::stopAll() {
  setWheels(0, 0);
  mPen->write(false);
  mMode = DriveMode::IDLE;
  mManualCmd = 'S';
}

// ---------------------------------------------------------------
// Commandes de l'IHM
// ---------------------------------------------------------------
void Servotron::handleCommand(const std::string &cmd) {
  if (cmd == "avancer" || cmd == "reculer" || cmd == "gauche" || cmd == "droite") {
    if (mMode != DriveMode::MANUAL) stopAll();
    mMode = DriveMode::MANUAL;
    mManualCmd = cmd == "avancer" ? 'F' : cmd == "reculer" ? 'B' : cmd == "gauche" ? 'L' : 'R';
  } else if (cmd == "stop") {
    stopAll();
  } else if (cmd.rfind("vitesse:", 0) == 0) {
    mSpeedPercent = clamp(std::atof(cmd.c_str() + 8), 0, 100);
  } else if (cmd.rfind("forme:", 0) == 0) {
    startShape(cmd.substr(6));
  } else if (cmd == "ligne") {
    startLine();
  } else if (cmd == "evitement") {
    startAvoid();
  }
}

void Servotron::startShape(const std::string &name) {
  mShape.clear();
  Pose2D start;
  if (name == "carre") {
    for (int i = 0; i < 4; ++i) {
      mShape.push_back({Segment::STRAIGHT, 0.6, 0});
      mShape.push_back({Segment::TURN, PI / 2, 0});
    }
    start = START_SQUARE;
    mShapeName = "carré";
  } else if (name == "triangle") {
    for (int i = 0; i < 3; ++i) {
      mShape.push_back({Segment::STRAIGHT, 0.7, 0});
      mShape.push_back({Segment::TURN, 2 * PI / 3, 0});
    }
    start = START_TRIANGLE;
    mShapeName = "triangle";
  } else if (name == "cercle") {
    mShape.push_back({Segment::ARC, 2 * PI, 0.35});
    start = START_CIRCLE;
    mShapeName = "cercle";
  } else {
    return;
  }

  stopAll();
  teleport(start);
  mShapeStart = start;
  mMode = DriveMode::SHAPE;
  mSegIndex = 0;
  mHeadingRef = start.yaw;
  mPrevHeadingError = 0;
  mSettleSteps = 0;
  mSegDistance = mSegTurned = 0;
  mPen->write(true);
}

void Servotron::startLine() {
  stopAll();
  teleport(START_LINE);
  mMode = DriveMode::LINE;
  mPrevLinePos = 0;
}

void Servotron::startAvoid() {
  stopAll();
  teleport(START_AVOID);
  mMode = DriveMode::AVOID;
  mAvoidTurning = false;
}

// ---------------------------------------------------------------
// Comportements
// ---------------------------------------------------------------
void Servotron::stepManual() {
  const double v = baseSpeed();
  switch (mManualCmd) {
    case 'F': setWheels(v, v); break;
    case 'B': setWheels(-v, -v); break;
    case 'L': setWheels(-0.5 * v, 0.5 * v); break;
    case 'R': setWheels(0.5 * v, -0.5 * v); break;
    default: setWheels(0, 0);
  }
}

void Servotron::stepShape() {
  if (mSegIndex >= mShape.size()) {
    stopAll();
    return;
  }
  const Segment &seg = mShape[mSegIndex];
  const double v = std::min(baseSpeed(), 9.0);
  bool done = false;

  switch (seg.type) {
    case Segment::STRAIGHT: {
      // Ligne droite : régulation PD sur le cap (odométrie) pour corriger la dérive
      const double err = wrapAngle(mHeadingRef - mTheta);
      const double corr = 8.0 * err + 40.0 * (err - mPrevHeadingError);
      mPrevHeadingError = err;
      const double remaining = seg.value - mSegDistance;
      const double scale = clamp(remaining / 0.12, 0.2, 1.0);
      setWheels(v * scale - corr, v * scale + corr);
      done = remaining <= 0.002;
      break;
    }
    case Segment::TURN: {
      // Rotation sur place : régulation proportionnelle de l'angle
      const double target = mHeadingRef + seg.value;
      const double err = wrapAngle(target - mTheta);
      double w = clamp(6.0 * err, -3.0, 3.0);
      if (std::fabs(w) < 0.4) w = std::copysign(0.4, err);
      setWheels(-w, w);
      if (std::fabs(err) < 0.004) {
        setWheels(0, 0);
        if (++mSettleSteps > 3) {
          mHeadingRef = wrapAngle(target);
          done = true;
        }
      } else {
        mSettleSteps = 0;
      }
      break;
    }
    case Segment::ARC: {
      // Arc de cercle : vitesses différentielles calculées depuis le rayon
      const double linear = v * WHEEL_RADIUS;  // m/s
      const double omega = linear / seg.radius;
      const double remaining = seg.value - mSegTurned;
      const double scale = clamp(remaining / 0.3, 0.25, 1.0);
      const double vl = (linear - omega * AXLE_LENGTH / 2) / WHEEL_RADIUS;
      const double vr = (linear + omega * AXLE_LENGTH / 2) / WHEEL_RADIUS;
      setWheels(vl * scale, vr * scale);
      done = remaining <= 0.002;
      if (done) mHeadingRef = wrapAngle(mHeadingRef + seg.value);
      break;
    }
  }

  if (done) {
    setWheels(0, 0);
    ++mSegIndex;
    mSegDistance = mSegTurned = 0;
    mPrevHeadingError = 0;
    mSettleSteps = 0;
  }
}

// Noirceur normalisée d'un capteur de sol : 0 = blanc, 1 = ligne noire
double Servotron::groundDarkness(int i) const {
  return clamp((mGroundWhite - mGround[i]->getValue()) / (mGroundWhite - mGroundBlack), 0.0, 1.0);
}

void Servotron::stepLine() {
  const double dl = groundDarkness(0), dc = groundDarkness(1), dr = groundDarkness(2);
  const double sum = dl + dc + dr;
  const double v = std::min(baseSpeed(), 8.0);

  if (sum < 0.25) {
    // Ligne perdue : on pivote du côté où elle a été vue en dernier
    setWheels(-0.35 * v * mLastLineSide, 0.35 * v * mLastLineSide);
    return;
  }

  // Position de la ligne : +1 = à gauche, -1 = à droite
  const double pos = (dl - dr) / sum;
  if (std::fabs(pos) > 0.2) mLastLineSide = pos > 0 ? 1 : -1;
  const double corr = v * (0.9 * pos + 2.5 * (pos - mPrevLinePos));
  mPrevLinePos = pos;
  const double speed = v * (1.0 - 0.45 * std::fabs(pos));
  setWheels(speed - corr, speed + corr);
}

void Servotron::stepAvoid() {
  const double dt = mTimeStep / 1000.0;
  const double front = mSonar->getValue();  // m
  const double kl = mIrLeft->getValue() / 1000.0;
  const double kr = mIrRight->getValue() / 1000.0;
  const double v = std::min(baseSpeed(), 9.0);

  if (mAvoidTurning) {
    mAvoidTimer += dt;
    if (mAvoidTimer < 0.5) {
      // Recul bref pour se dégager avant de pivoter
      setWheels(-0.5 * v, -0.5 * v);
    } else {
      setWheels(-0.45 * v * mAvoidDir, 0.45 * v * mAvoidDir);
      if (front > 0.6 && kl < 0.2 && kr < 0.2 && mAvoidTimer > 0.9) mAvoidTurning = false;
    }
    return;
  }

  // Obstacle devant, trop proche sur un côté, ou couloir trop étroit
  if (front < 0.38 || kl > 0.6 || kr > 0.6 || (kl > 0.35 && kr > 0.35)) {
    mAvoidTurning = true;
    mAvoidTimer = 0;
    if (std::fabs(kl - kr) > 0.05)
      mAvoidDir = kl > kr ? -1 : 1;  // obstacle à gauche -> on tourne à droite
    else
      mAvoidDir = (std::rand() % 2) ? 1 : -1;
    return;
  }

  // Braitenberg : chaque capteur IR repousse le robot de son côté
  const double slow = clamp(front / 0.7, 0.45, 1.0);
  const double left = v * slow * (1.0 + 1.3 * kl - 1.3 * kr);
  const double right = v * slow * (1.0 + 1.3 * kr - 1.3 * kl);
  setWheels(left, right);
}

// ---------------------------------------------------------------
// Boucle principale
// ---------------------------------------------------------------
void Servotron::run() {
  step(mTimeStep);
  resetOdometry({getSelf()->getPosition()[0], getSelf()->getPosition()[1], 0.0});

  while (step(mTimeStep) != -1) {
    updateOdometry();

    for (std::string msg = wwiReceiveText(); !msg.empty(); msg = wwiReceiveText())
      handleCommand(msg);
    if (mTest) testStep(getTime());

    switch (mMode) {
      case DriveMode::MANUAL: stepManual(); break;
      case DriveMode::SHAPE: stepShape(); break;
      case DriveMode::LINE: stepLine(); break;
      case DriveMode::AVOID: stepAvoid(); break;
      default: setWheels(0, 0);
    }
  }
}

// ---------------------------------------------------------------
// Mode test de développement : enchaîne les démonstrations et affiche des mesures.
// Inactif en utilisation normale.
// ---------------------------------------------------------------
void Servotron::testStep(double t) {
  static int phase = 0;
  static double phaseStart = 0, lineSteps = 0, lineSeen = 0, lineDist = 0;
  static double lastX = 0, lastY = 0, stuckTime = 0, minFront = 99;
  const double *p = getSelf()->getPosition();

  auto next = [&](const std::string &cmd) {
    ++phase;
    phaseStart = t;
    lastX = p[0];
    lastY = p[1];
    if (!cmd.empty()) handleCommand(cmd);
  };

  switch (phase) {
    case 0:
      if (t > 0.3) next("ligne");
      break;
    case 1: {  // suivi de ligne pendant 70 s
      lineSteps++;
      if (groundDarkness(0) + groundDarkness(1) + groundDarkness(2) > 0.25) lineSeen++;
      lineDist += std::hypot(p[0] - lastX, p[1] - lastY);
      lastX = p[0];
      lastY = p[1];
      if (t - phaseStart > 70) {
        std::printf("[TEST] ligne : %.1f m parcourus, ligne vue %.0f%% du temps\n", lineDist, 100.0 * lineSeen / lineSteps);
        next("forme:carre");
      }
      break;
    }
    case 2:
    case 3:
    case 4:
      if (mMode == DriveMode::IDLE || t - phaseStart > 60) {
        const double *R = getSelf()->getOrientation();
        std::printf("[TEST] %s : %s en %.1f s, écart de fermeture %.1f cm, cap %.1f°\n", mShapeName.c_str(),
                    mMode == DriveMode::IDLE ? "terminé" : "DÉLAI DÉPASSÉ", t - phaseStart,
                    100 * std::hypot(p[0] - mShapeStart.x, p[1] - mShapeStart.y),
                    std::atan2(R[3], R[0]) * 180 / PI);
        next(phase == 2 ? "forme:triangle" : phase == 3 ? "forme:cercle" : "evitement");
      }
      break;
    case 5: {  // évitement pendant 90 s
      minFront = std::min(minFront, mSonar->getValue());
      if (std::hypot(p[0] - lastX, p[1] - lastY) < 0.0005 && !mAvoidTurning) stuckTime += mTimeStep / 1000.0;
      lastX = p[0];
      lastY = p[1];
      if (t - phaseStart > 90) {
        std::printf("[TEST] évitement : sonar min %.2f m, temps bloqué %.1f s\n", minFront, stuckTime);
        std::fflush(stdout);
        simulationQuit(0);
      }
      break;
    }
  }
  std::fflush(stdout);
}

}  // namespace

int main() {
  Servotron robot;
  robot.run();
  return 0;
}
