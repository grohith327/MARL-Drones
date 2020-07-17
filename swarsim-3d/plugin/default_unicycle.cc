#include "default_unicycle.h"

#include <cmath>

#include "util/utils.h"

namespace {
constexpr double kDistanceThreshold = 0.2;
constexpr double kGoalRadius = 0.05;
}  // namespace

bool DefaultUnicycle::Initialize() {
  // Pick a random goal point .
  goal_x_ = RandomUniform() * 3.0 - 1.5;
  goal_y_ = RandomUniform() * 3.0 - 1.5;
  return true;
}

void DefaultUnicycle::Execute(double t, double dt) {
  double dx = goal_x_ - x();
  double dy = goal_y_ - y();
  double e = sqrtf(dx * dx + dy * dy);
  while (e < kDistanceThreshold) {
    goal_x_ = RandomUniform() * 3.0 - 1.5;
    goal_y_ = RandomUniform() * 3.0 - 1.5;
    dx = goal_x_ - x();
    dy = goal_y_ - y();
    e = sqrtf(dx * dx + dy * dy);
  }
  // Go to goal point.
  SetGoal(goal_x_, goal_y_);
}

void DefaultUnicycle::Draw(VisualizerWindow* window) {
  Unicycle::Draw(window);

  glPushMatrix();
  glTranslatef(goal_x_, 0.0, -goal_y_);
  glRotatef(90.0, 1.0, 0.0, 0.0);
  // Draw goal point.
  GLUquadricObj* qobj = gluNewQuadric();
  gluQuadricDrawStyle(qobj, GLU_FILL);
  gluQuadricOrientation(qobj, GLU_INSIDE);
  auto robot_color = color();
  glColor3f(std::get<0>(robot_color) * 0.5, std::get<1>(robot_color) * 0.5, std::get<2>(robot_color) * 0.5);
  gluDisk(qobj, 0, kGoalRadius, 32, 5);
  gluDeleteQuadric(qobj);
  glPopMatrix();
}
