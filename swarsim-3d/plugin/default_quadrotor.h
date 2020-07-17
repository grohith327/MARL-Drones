#ifndef _DEFAULT_QUADROTOR_H
#define _DEFAULT_QUADROTOR_H

#include <iostream>
#include <vector>

#include "core/quadrotor.h"
#include "core/robot.h"
#include "display/window.h"
#include "util/registerer.h"

class DefaultQuadrotor : public Quadrotor {
  REGISTER("DefaultQuadrotor", Robot);

 public:
  DefaultQuadrotor();
  void Draw(VisualizerWindow* window) override;

  std::vector<std::vector<int>> positions;
  int count;

 private:
  bool Initialize(int robot_type) override;
  void Execute(double t, double dt, int robot_type) override;
  bool GoalBased() override { return true; }

  double goal_x_;
  double goal_y_;
  double goal_z_;

};

#endif
