#ifndef _DEFAULT_UNICYCLE_H
#define _DEFAULT_UNICYCLE_H

#include "core/unicycle.h"
#include "core/robot.h"
#include "display/window.h"
#include "util/registerer.h"

class DefaultUnicycle : public Unicycle {
  REGISTER("DefaultUnicycle", Robot);

 public:
  void Draw(VisualizerWindow* window) override;

 private:
  bool Initialize() override;
  void Execute(double t, double dt) override;
  bool GoalBased() override { return true; }

  double goal_x_;
  double goal_y_;
};

#endif
