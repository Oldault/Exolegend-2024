#include "gladiator.h"
#include <cmath>
#include <math.h>
#include <chrono>
#include <thread>
#include <algorithm>
#undef abs

// x,y représentent des coordonnées en m
// Vector{1.5,1.5} représente le point central
// Pour convertir une cordonnée de cellule (i,j) (0<=i<=13, 0<=j<=13) :
// x = i * CELL_SIZE + 0.5*CELL_SIZE
// y = j * CELL_SIZE + 0.5*CELL_SIZE
// avec CELL_SIZE = 3.0/14 (~0.214)

class Vector2
{
public:
  Vector2() : _x(0.), _y(0.) {}
  Vector2(float x, float y) : _x(x), _y(y) {}

  float norm1() const { return std::abs(_x) + std::abs(_y); }
  float norm2() const { return std::sqrt(_x * _x + _y * _y); }
  void normalize()
  {
    _x /= norm2();
    _y /= norm2();
  }
  Vector2 normalized() const
  {
    Vector2 out = *this;
    out.normalize();
    return out;
  }

  Vector2 operator-(const Vector2 &other) const { return {_x - other._x, _y - other._y}; }
  Vector2 operator+(const Vector2 &other) const { return {_x + other._x, _y + other._y}; }
  Vector2 operator*(float f) const { return {_x * f, _y * f}; }

  bool operator==(const Vector2 &other) const { return std::abs(_x - other._x) < 1e-5 && std::abs(_y - other._y) < 1e-5; }
  bool operator!=(const Vector2 &other) const { return !(*this == other); }

  float x() const { return _x; }
  float y() const { return _y; }

  float dot(const Vector2 &other) const { return _x * other._x + _y * other._y; }
  float cross(const Vector2 &other) const { return _x * other._y - _y * other._x; }
  float angle(const Vector2 &m) const { return std::atan2(cross(m), dot(m)); }
  float angle() const { return std::atan2(_y, _x); }

private:
  float _x, _y;
};

Gladiator *gladiator;

inline float moduloPi(float a) // return angle in [-pi; pi]
{
  return (a < 0.0) ? (std::fmod(a - M_PI, 2 * M_PI) + M_PI) : (std::fmod(a + M_PI, 2 * M_PI) - M_PI);
}

inline bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs, bool toMove, float speed)
{
  constexpr float ANGLE_REACHED_THRESHOLD = 0.1;
  constexpr float POS_REACHED_THRESHOLD = 0.05;

  auto posRaw = gladiator->robot->getData().position;
  Vector2 pos{posRaw.x, posRaw.y};

  Vector2 posError = target - pos;

  float targetAngle = posError.angle();
  float angleError = moduloPi(targetAngle - posRaw.a);

  bool targetReached = false;
  float leftCommand = 0.f;
  float rightCommand = 0.f;

  if (posError.norm2() < POS_REACHED_THRESHOLD) //
  {
    targetReached = true;
  }
  else if (std::abs(angleError) > ANGLE_REACHED_THRESHOLD)
  {
    float factor = 0.1;
    if (angleError < 0)
      factor = -factor;
    rightCommand = factor;
    leftCommand = -factor;
  }
  else
  {
    if (toMove)
    {
      // gladiator->log("Moving");
      float factor = speed;
      rightCommand = factor; //+angleError*0.1  => terme optionel, "pseudo correction angulaire";
      leftCommand = factor;  //-angleError*0.1   => terme optionel, "pseudo correction angulaire";
    }
    else
    {
      // gladiator->log("Robot Id %d AIMED", gladiator->robot->getData().id);
      gladiator->weapon->launchRocket();
      rightCommand = 0;
      leftCommand = 0;
    }
  }

  gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
  gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

  if (showLogs)
  {
    gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError, target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
  }

  return targetReached;
}

void show_Logs(int elapsedTime, int x, int y, int teamId, char *direction)
{
  gladiator->log("-------------------------------------");
  gladiator->log("Time since start: %d", elapsedTime);
  gladiator->log("Square Data x = %d, y = %d", x, y);
  gladiator->log("Gladiator team : %d", teamId);
  gladiator->log("Going : %s", direction);
}

void moveSouth(const Position &position, unsigned i, float delta, float &dx, float &dy)
{
  dy = position.y - delta;
  dx = position.x + delta / 10 * (i % 2) * (-1);
}

void moveEast(const Position &position, unsigned i, float delta, float &dx, float &dy)
{
  dx = position.x + delta;
  dy = position.y + delta / 10 * (i % 2) * (-1);
}

void moveNorth(const Position &position, unsigned i, float delta, float &dx, float &dy)
{
  dy = position.y + delta;
  dx = position.x + delta / 10 * (i % 2) * (-1);
}

void moveWest(const Position &position, unsigned i, float delta, float &dx, float &dy)
{
  dx = position.x - delta;
  dy = position.y + delta / 10 * (i % 2) * (-1);
}

// Directional Movement Logic
void moveTowardsDirection(const MazeSquare *square, const Position &position, unsigned i, float delta, float &dx, float &dy, long elapsedTime, char *direction)
{
  if (strcmp(direction, (char *)"SOUTH") == 0)
    moveSouth(position, i, delta, dx, dy);
  else if (strcmp(direction, (char *)"EAST") == 0)
    moveEast(position, i, delta, dx, dy);
  else if (strcmp(direction, (char *)"NORTH") == 0)
    moveNorth(position, i, delta, dx, dy);
  else if (strcmp(direction, (char *)"WEST") == 0)
    moveWest(position, i, delta, dx, dy);
  else
  {
    dx = 1.5 + (elapsedTime % 3) * 0.1;
    dy = 1.5 - (elapsedTime % 3) * 0.1;
  }
  show_Logs(elapsedTime, square->i, square->j, gladiator->robot->getData().teamId, direction);
}

// Time and Position Adjustment
void adjustTimeAndPosition(long elapsedTime, float &PMin, float &PMax, int &TimeFlag, float &delta)
{
  if (elapsedTime > 0 && !((elapsedTime + 1) % 20) && TimeFlag)
  {
    PMin = PMin + delta;
    PMax = PMax - delta;
    gladiator->log("in time Pmin = %lf", PMin);
    gladiator->log("in time Pmax = %lf", PMax);
    TimeFlag = 0;
  }
  if ((elapsedTime + 1) % 20)
    TimeFlag = 1;
}

// Check Position Bounds and Center Positioning
void checkBoundsAndCenterPosition(float &dx, float &dy, long elapsedTime, const float PMin, const float PMax, const float epsilon)
{
  if (((dx + epsilon) < PMin || (dx - epsilon) > PMax) || ((dy + epsilon) < PMin || (dy - epsilon) > PMax))
  {
    dx = 1.5;
    dy = 1.5;
    gladiator->log("in bounds Pmin = %f", PMin);
    gladiator->log("in bounds Pmax = %f", PMax);
    show_Logs(elapsedTime, gladiator->maze->getNearestSquare()->i, gladiator->maze->getNearestSquare()->j, gladiator->robot->getData().teamId, (char *)"BOUNDS");
  }
}

// Launch Rocket if Applicable
// void checkAndLaunchRocket(float dx, float dy, RobotData &enemyData1, RobotData &enemyData2, RobotData &data)
// {
//   if (gladiator->weapon->canLaunchRocket())
//   {
//     int x1 = (enemyData1.position.x - data.position.x);
//     int y1 = (enemyData1.position.y - data.position.y);
//     int x2 = (enemyData2.position.x - data.position.x);
//     int y2 = (enemyData2.position.y - data.position.y);

//     int dist1 = sqrt((x1 * x1) + (y1 * y1));
//     int dist2 = sqrt((x2 * x2) + (y2 * y2));

//     int dmin = std::min(dist1, dist2);
//     if (dmin == dist1)
//       aim();
//     else
//       aim();

//     gladiator->weapon->launchRocket();
//   }
// }

void getEnemyIDs(uint8_t *ids, int size, int &enemyID1, int &enemyID2, int myID)
{
  int found = 0; // Keep track of how many enemy IDs have been found
  for (int i = 0; i < size; ++i)
  {
    if (ids[i] != 110 && ids[i] != 12 && ids[i] != myID)
    {
      if (found == 0)
      {
        enemyID1 = ids[i];
        ++found;
      }
      else if (found == 1)
      {
        enemyID2 = ids[i];
        break;
      }
    }
  }
}

void reset();
void setup()
{
  // instanciation de l'objet gladiator
  gladiator = new Gladiator();
  // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
  gladiator->game->onReset(&reset);
}

void reset()
{
  // fonction de reset:
  // initialisation de toutes vos variables avant le début d'un match
  gladiator->log("Robot reset successfully");
}

void loop()
{
  static unsigned i = 0;
  float dx{0};
  float dy{0};
  float delta = 3.0 / 12;
  float epsilon = 0.015;

  float PMin = 0.05;
  float PMax = 2.95;

  int TimeFlag = 0;

  auto start = std::chrono::steady_clock::now();

  RobotList robotList = gladiator->game->getPlayingRobotsId();
  int enemyID1 = -1, enemyID2 = -1;
  RobotData data = gladiator->robot->getData();
  getEnemyIDs(robotList.ids, 4, enemyID1, enemyID2, data.id);

  while (gladiator->game->isStarted())
  {
    // Our Gladiator info
    data = gladiator->robot->getData();
    Position position = data.position;
    const MazeSquare *square = gladiator->maze->getNearestSquare();

    // Enemies
    RobotData enemyData1 = gladiator->game->getOtherRobotData(enemyID1);
    RobotData enemyData2 = gladiator->game->getOtherRobotData(enemyID2);

    auto currentTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - start).count();

    adjustTimeAndPosition(elapsedTime, PMin, PMax, TimeFlag, delta);

    float delta1 = delta * 0.9;
    if (square->southSquare != nullptr && data.teamId != square->southSquare->possession)
      moveTowardsDirection(square, position, i, delta1, dx, dy, elapsedTime, (char *)"SOUTH");
    else if (square->eastSquare != nullptr && data.teamId != square->eastSquare->possession)
      moveTowardsDirection(square, position, i, delta1, dx, dy, elapsedTime, (char *)"EAST");
    else if (square->northSquare != nullptr && data.teamId != square->northSquare->possession)
      moveTowardsDirection(square, position, i, delta1, dx, dy, elapsedTime, (char *)"NORTH");
    else if (square->westSquare != nullptr && data.teamId != square->westSquare->possession)
      moveTowardsDirection(square, position, i, delta1, dx, dy, elapsedTime, (char *)"WEST");
    else
      moveTowardsDirection(square, position, i, delta1, dx, dy, elapsedTime, (char *)"CENTER");

    if (((dx + epsilon) < PMin || (dx - epsilon) > PMax) || ((dy + epsilon) < PMin || (dy - epsilon) > PMax))
    {
      gladiator->log("dy = %f", dy);
      gladiator->log("dx = %f", dx);
      gladiator->log("Pmin = %f", PMin);
      gladiator->log("Pmax = %f", PMax);
      dy = 1.5;
      dx = 1.5;
      show_Logs(elapsedTime, square->i, square->j, data.teamId, (char *)"BOUNDS");
    }

    // Get Distance
    float x1 = (enemyData1.position.x - data.position.x);
    float y1 = (enemyData1.position.y - data.position.y);
    float x2 = (enemyData2.position.x - data.position.x);
    float y2 = (enemyData2.position.y - data.position.y);

    float dist1 = sqrt((x1 * x1) + (y1 * y1));
    float dist2 = sqrt((x2 * x2) + (y2 * y2));

    float dmin = std::min(dist1, dist2);

    // gladiator->log("----------------------------------------");
    // gladiator->log("Enemy data 1 = x:%f, y:%f", enemyData1.position.x, enemyData1.position.y);
    // gladiator->log("Enemy data 2 = x:%f, y:%f", enemyData2.position.x, enemyData2.position.y);
    // gladiator->log("My = x:%f, y:%f", data.position.x, data.position.y);
    // gladiator->log("dist1 : %f, dist2 : %f", dist1, dist2);

    // Shoot or walk
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    if (gladiator->weapon->canLaunchRocket() && dmin < (float)(15.0 / 12.0))
    {
      if (dist1 < dist2)
      {
        gladiator->log("FIRING ROCKET AT ROBOT %d. DITANCE : %f", enemyData1.id, dist1);
        gladiator->log("Enemy Robot at x:%f, y:%f", x1, y1);
        while (std::chrono::steady_clock::now() < end_time)
        {
          aim(gladiator, {enemyData1.position.x, enemyData1.position.y}, 0, 0, 0.5);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
      else
      {
        gladiator->log("FIRING ROCKET AT ROBOT %d. DITANCE : %f", enemyData2.id, dist2);
        gladiator->log("Enemy Robot at x:%f, y:%f", x2, y2);
        while (std::chrono::steady_clock::now() < end_time)
        {
          aim(gladiator, {enemyData2.position.x, enemyData2.position.y}, 0, 0, 0.5);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    }
    else
    {
      while (std::chrono::steady_clock::now() < end_time)
      {
        if (dx == 1.5 && dy == 1.5)
          aim(gladiator, {dx, dy}, 0, 1, 1);
        else
        {
          if (aim(gladiator, {dx, dy}, 0, 1, 0.3))
          {
            gladiator->log("target atteinte !");
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Short sleep to prevent overwhelming the system
      }
    }

    // checkAndLaunchRocket(dx, dy, enemyData1, enemyData2, data);

    i++;
    delay(10);
  }
}