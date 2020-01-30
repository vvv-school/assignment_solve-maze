// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Assignment on Solving a Maze.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/cv/Cv.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/pids.h>

#define MAZE_L  500

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::cv;
using namespace iCub::ctrl;


class GraphicObject {
 public:
  virtual Bottle getInfo() const = 0;
  virtual void draw(ImageOf<PixelRgb>& img) const = 0;
};


class CircleObject : public GraphicObject {
 protected:
  Vector c;
  double r;
  cv::Scalar bound_color, fill_color;

 public:
  CircleObject(const Vector& center, const double radius) :
      c(center), r(radius),
      bound_color(255, 255, 255),
      fill_color(0, 0, 0) { }

  Bottle getInfo() const override {
    Vector v = cat(c, r);
    Bottle info;
    info.addList().read(v);
    return info;
  }

  void draw(ImageOf<PixelRgb>& img) const override {
    cv::Point c_((int)c[0], (int)(img.height() - c[1]));
    int r_ = (int)r;

    cv::Mat imgMat = toCvMat(img);
    cv::circle(imgMat, c_, r_, fill_color, cv::FILLED);
    cv::circle(imgMat, c_, r_, bound_color, 1);
  }

  bool inside(const Vector& point) const {
    return (norm(point - c) < r);
  }
};


class Obstacle : public CircleObject {
 public:
  Obstacle(const Vector& center, const double radius) :
      CircleObject(center, radius) {
    fill_color = cv::Scalar(249, 196, 71);
  }
};


class Target : public CircleObject {
 public:
  Target(const Vector& center, const double radius) :
      CircleObject(center, radius) {
    fill_color = cv::Scalar(72, 208, 154);
  }
};


class Craft : public GraphicObject {
  double r;
  cv::Scalar color;

  vector<Vector> points;

  Filter* F;
  Integrator* I;

 public:
  Craft(const Vector& center, const double radius,
        const double Ts) :
      r(radius), color(96, 176, 224), points(3, Vector(3, 1.0)) {
    double tau = 0.5;
    double alpha = (2.0 * tau) / Ts;

    Vector num(2, 1.0), den(2);
    den[0] = 1.0 + alpha;
    den[1] = 1.0 - alpha;

    F = new Filter(num, den, Vector(2, 0.0));
    I = new Integrator(Ts, cat(center, 0.0));

    points[0][0] = r * cos(0.0 * CTRL_DEG2RAD);
    points[0][1] = r * sin(0.0 * CTRL_DEG2RAD);

    points[1][0] = r * cos(150.0 * CTRL_DEG2RAD);
    points[1][1] = r * sin(150.0 * CTRL_DEG2RAD);

    points[2][0] = r * cos(210.0 * CTRL_DEG2RAD);
    points[2][1] = r * sin(210.0 * CTRL_DEG2RAD);
  }

  void move(const Vector& velocity) {
    Vector v(2);
    v[0] = 0.3 * std::min(std::max(velocity[0], -100.0), 100.0);
    v[1] = 0.6 * std::min(std::max(velocity[1], -100.0), 100.0);
    v = F->filt(v);

    Vector curState = I->get();
    double dirRad = curState[2] * CTRL_DEG2RAD;

    Vector vel(3);
    vel[0] = v[0] * cos(dirRad);
    vel[1] = v[0] * sin(dirRad);
    vel[2] = v[1];

    I->integrate(vel);
  }

  Bottle getInfo() const override {
    Bottle info;
    Vector curState = I->get();
    curState[2] = fmod(curState[2], 360.0);
    if (curState[2] < 0.0)
      curState[2] += 360.0;
    info.addList().read(curState);
    return info;
  }

  void draw(ImageOf<PixelRgb>& img) const override {
    Vector curState = I->get();
    Vector rot(4, 0.0);
    rot[2] = 1.0;
    rot[3] = curState[2] * CTRL_DEG2RAD;
    Matrix T = axis2dcm(rot).submatrix(0, 2, 0, 2);
    T(0, 2) = curState[0];
    T(1, 2) = curState[1];

    vector<cv::Point> pts;
    for (auto& point:points) {
      Vector p = T * point;
      pts.push_back(cv::Point((int)p[0], (int)(img.height() - p[1])));
    }

    vector<vector<cv::Point>> poly(1, pts);
    cv::Mat imgMat = toCvMat(img);
    cv::fillPoly(imgMat, poly, color);
  }

  virtual ~Craft() {
    delete F;
    delete I;
  }
};


class HandlerModule : public RFModule {
  double timeBudget,t0;

  Target* target;
  Craft* craft;
  vector<Obstacle*> obstacles;
  bool running;

  BufferedPort<ImageOf<PixelRgb>> portMaze;
  BufferedPort<Bottle> portMotor;
  BufferedPort<Property> portRadar;

  Vector velocity;

  bool sendRadar(const double t, const ImageOf<PixelRgb>& img) {
    string state = "running";

    Bottle craftInfo = *craft->getInfo().get(0).asList();
    Vector craftPos(2);
    craftPos[0] = craftInfo.get(0).asDouble();
    craftPos[1] = craftInfo.get(1).asDouble();

    Property& radar = portRadar.prepare();
    radar.clear();

    radar.put("length", MAZE_L);
    radar.put("time", t);
    radar.put("craft", craft->getInfo().get(0));
    radar.put("target", target->getInfo().get(0));

    if (target->inside(craftPos))
      state = "solved";

    Bottle obstaclesInfo;
    Bottle& obstacles_ = obstaclesInfo.addList();
    for (auto& obstacle:obstacles) {
      obstacles_.append(obstacle->getInfo());
      if (obstacle->inside(craftPos))
        state = "crashed";
    }
    radar.put("obstacles", obstaclesInfo.get(0));

    if ((craftPos[0] < 0) || (craftPos[0] >= img.width()) ||
        (craftPos[1] < 0) || (craftPos[1] >= img.height()))
      state = "crashed";

    if (t > timeBudget)
      state = "expired";
    radar.put("state", state);

    portRadar.writeStrict();
    return (state == "running");
  }

 public:
  bool configure(ResourceFinder& rf)override {
    timeBudget = rf.check("time-budget", Value(numeric_limits<double>::infinity())).asDouble();
    Rand::init();

    Vector target_c(2, MAZE_L - 50.0);
    Vector craft_c(2, 50.0);
    target = new Target(target_c, 4.0);
    craft = new Craft(craft_c, 7.0, getPeriod());
    for (int i = 0; i < 20; i++) {
      Vector obstacle_c;
      do
        obstacle_c = Rand::vector(Vector(2, 0.0), Vector(2, MAZE_L));
      while ((norm(obstacle_c - target_c) < 60.0) || (norm(obstacle_c - craft_c) < 60.0));
      obstacles.push_back(new Obstacle(obstacle_c, Rand::scalar(20.0, 50.0)));
    }

    portMaze.open("/assignment_solve-maze-handler/maze:o");
    portMotor.open("/assignment_solve-maze-handler/motor:i");
    portRadar.open("/assignment_solve-maze-handler/radar:o");

    velocity.resize(2, 0.0);
    running = true;
    t0 = Time::now();

    return true;
  }

  bool close()override {
    for (auto& obstacle:obstacles)
      delete obstacle;
    delete craft;
    delete target;

    portMaze.close();
    portMotor.close();
    portRadar.close();

    return true;
  }

  double getPeriod()override {
    return 0.033;
  }

  bool updateModule()override {
    if (running) {
      double t = Time::now() - t0;
      if (Bottle* cmd = portMotor.read(false)) {
        if (cmd->size() >= 2) {
          velocity[0] = cmd->get(0).asDouble();
          velocity[1] = cmd->get(1).asDouble();
        }
      }

      ImageOf<PixelRgb>& maze = portMaze.prepare();
      maze.resize(MAZE_L, MAZE_L);
      maze.zero();

      craft->move(velocity);

      for (auto& obstacle:obstacles)
        obstacle->draw(maze);
      target->draw(maze);
      craft->draw(maze);

      portMaze.writeStrict();
      running = sendRadar(t, maze);
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

  HandlerModule mod;
  return mod.runModule(rf);
}
