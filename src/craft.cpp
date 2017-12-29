// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Assignment on Solving a Maze.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <limits>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#define DEG2RAD     (M_PI/180.0)
#define RAD2DEG     (180.0/M_PI)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


enum class State {
  idle,
  pickup_direction,
  rotating,
  moving,
  on_target
};


class SteadyStateChecker {
  deque<double> fifo;
  unsigned int length;

 public:
  SteadyStateChecker() : length(5) { }

  void push(const double item) {
    fifo.push_back(item);
    if (fifo.size() > length)
      fifo.pop_front();
  }

  void clear() {
    fifo.clear();
  }

  bool check(const double threshold) const {
    if (fifo.size() < length)
      return false;
    for (auto& item:fifo)
      if (abs(item) > threshold)
        return false;
    return true;
  }
};


class CraftModule : public RFModule {
  BufferedPort<Bottle> portMotor;
  BufferedPort<Property> portRadar;

  Vector craft;
  Vector target;
  Vector wayPoint;
  vector<Vector> obstacles;
  vector<Vector> walls;
  double direction;
  double safety_margin;

  SteadyStateChecker steady;
  State state;

  void getRadar() {
    if (Property* radar = portRadar.read(false)) {
      if (Bottle* b = radar->find("craft").asList())
        for (int i = 0; i < b->size(); i++)
          craft[i] = b->get(i).asDouble();

      if (Bottle* b = radar->find("target").asList())
        for (int i = 0; i < b->size(); i++)
          target[i] = b->get(i).asDouble();
      target[0] -= craft[0];
      target[1] -= craft[1];

      if (Bottle* b = radar->find("obstacles").asList()) {
        obstacles.clear();
        for (int i = 0; i < b->size(); i++) {
          if (Bottle* o = b->get(i).asList()) {
            Vector obstacle(o->size());
            for (int j = 0; j < o->size(); j++)
              obstacle[j] = o->get(j).asDouble();
            obstacle[0] -= craft[0];
            obstacle[1] -= craft[1];
            obstacles.push_back(obstacle);
          }
        }
      }

      double length = radar->find("length").asDouble();
      walls[0][0] = length - craft[0];
      walls[0][1] = 0.0;
      walls[1][0] = 0.0;
      walls[1][1] = length - craft[1];
      walls[2][0] = -craft[0];
      walls[2][1] = 0.0;
      walls[3][0] = 0.0;
      walls[3][1] = -craft[1];

      if (state == State::idle)
        state = State::pickup_direction;
    }
  }

  Vector computeCrashPoint(const double direction) const {
    Vector dir(2);
    dir[0] = cos(direction);
    dir[1] = sin(direction);

    Vector crashPoint(2, numeric_limits<double>::infinity());
    for (auto& obstacle:obstacles) {
      Vector v = obstacle.subVector(0, 1);
      double d = norm(v);
      double r = obstacle[2];

      double cos_theta = dot(dir, v) / d;
      if (acos(cos_theta) <= asin(r / d)) {
        double dist = d * cos_theta - sqrt(r * r - d * d * (1.0 - cos_theta * cos_theta));
        if (norm(crashPoint) > dist)
          crashPoint = dist * dir;
      }
    }

    for (auto& wall:walls) {
      double d = norm(wall);
      double cos_theta = dot(dir, wall) / d;
      if (cos_theta > 0.0) {
        double theta = acos(cos_theta);
        double tan_theta = tan(theta);
        double dist = d * sqrt(1.0 + tan_theta * tan_theta);
        if (norm(crashPoint) > dist)
          crashPoint = dist * dir;
      }
    }

    return crashPoint;
  }

  bool rotate() {
    Bottle& motor = portMotor.prepare();
    motor.clear();

    double error = RAD2DEG * direction - craft[2];
    double vel = std::min(std::max(error, -100.0), 100.0);

    motor.addDouble(0.0);
    motor.addDouble(vel);
    portMotor.writeStrict();

    steady.push(error);
    return steady.check(0.1);
  }

  bool move() {
    Bottle& motor = portMotor.prepare();
    motor.clear();

    Vector dir(2);
    dir[0] = cos(direction);
    dir[1] = sin(direction);

    Vector dist = wayPoint - craft.subVector(0, 1);

    double error = sign(dot(dir, dist)) * norm(dist);
    double vel = std::min(std::max(error, -100.0), 100.0);

    motor.addDouble(vel);
    motor.addDouble(0.0);
    portMotor.writeStrict();

    steady.push(error);
    return steady.check(target[2]);
  }

  void stop() {
    Bottle& motor = portMotor.prepare();
    motor.clear();
    motor.addDouble(0.0);
    motor.addDouble(0.0);
    portMotor.writeStrict();
  }

 public:
  bool configure(ResourceFinder& rf)override {
    Rand::init();

    craft.resize(3, 0.0);
    target.resize(3, 0.0);
    wayPoint.resize(2, 0.0);
    walls.assign(4, Vector(2, 0.0));
    direction = 0.0;
    safety_margin = 20.0;

    state = State::idle;

    portMotor.open("/assignment_solve-maze-craft/motor:o");
    portRadar.open("/assignment_solve-maze-craft/radar:i");

    return true;
  }

  bool close()override {
    portMotor.close();
    portRadar.close();

    return true;
  }

  double getPeriod()override {
    return 0.02;
  }

  bool updateModule()override {
    getRadar();

    if (state == State::pickup_direction) {
      double direction_candidate = atan2(target[1], target[0]);
      if (direction_candidate < 0.0)
        direction_candidate += 2.0 * M_PI;
      Vector crashPoint = computeCrashPoint(direction_candidate);
      yInfo() << "test direction to target [deg] =" << RAD2DEG * direction_candidate;
      double d = norm(crashPoint);
      if (d > safety_margin)
        state = State::rotating;
      else {
        direction_candidate = Rand::scalar(0.0, 2.0 * M_PI);
        crashPoint = computeCrashPoint(direction_candidate);
        yInfo() << "test random direction [deg] =" << RAD2DEG * direction_candidate;
        d = norm(crashPoint);
        if (d > safety_margin)
          state = State::rotating;
      }

      if (state == State::rotating) {
        direction = direction_candidate;
        wayPoint = craft.subVector(0, 1) + ((d - 0.5 * safety_margin) / d) * crashPoint;
        steady.clear();
        yInfo() << "set way-point = (" << wayPoint.toString(3, 3) << ")";
      }
    } else if (state == State::rotating) {
      if (rotate()) {
        stop();
        steady.clear();
        state = State::moving;
      }
    } else if (state == State::moving) {
      if (move()) {
        stop();
        state = (norm(target.subVector(0, 1)) < target[2]) ?
            State::on_target : State::pickup_direction;
      }
    }

    return true;
  }
};


int main(int argc, char* argv[]) {
  Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "YARP doesn't seem to be available";
    return EXIT_FAILURE;
  }

  ResourceFinder rf;
  rf.configure(argc, argv);

  CraftModule mod;
  return mod.runModule(rf);
}
