#ifndef _UNICYCLE_H
#define _UNICYCLE_H

#include <memory>

#include "display/model_3ds.h"
#include "display/window.h"
#include "robot.h"

class Unicycle : public Robot {
 public:
  Unicycle();

  // Initializes the robots.
  bool Init() final;

  // Executes a step.
  void Step(double t, double dt) final;

  // Draw the robot.
  void Draw(VisualizerWindow* window) override;

  bool IsGround() const final { return true; };
  bool IsFlying() const final { return false; };

  // Control inputs.
  double u() const { return u_; }
  double w() const { return w_; }

 protected:
  // Sets the control inputs for the next time step.
  void SetControlInputs(double u, double w);

  // Sets the goal position if GoalBased returns true.
  void SetGoal(double x, double y);

 private:
  // Control inputs.
  double u_;
  double w_;

  // Goal position.
  double goal_x_;
  double goal_y_;

  // These are static, we are guaranteed the Draw() is called sequentially.
  static bool lists_created_;
  static std::unique_ptr<Model3DS> model_;
  static GLuint robot_dl_;
};

#endif
