#ifndef _BICYCLE_H
#define _BICYCLE_H

#include <memory>

#include "display/model_3ds.h"
#include "display/window.h"
#include "robot.h"

class Bicycle : public Robot {
 public:
  Bicycle();

  // Initializes the robots.
  bool Init() final;

  // Executes a step.
  void Step(double t, double dt) final;

  // Draw the robot.
  void Draw(VisualizerWindow* window) override;

  bool IsGround() const final { return true; };
  bool IsFlying() const final { return false; };

  // Control inputs.
  double v() const { return v_; }
  double phi() const { return phi_; }

 protected:
  // Sets the control inputs for the next time step.
  void SetControlInputs(double v, double phi);

  // Sets the goal position if GoalBased returns true.
  void SetGoal(double x, double y);

 private:
  // Control inputs.
  double v_;
  double phi_;

  // Goal position.
  double goal_x_;
  double goal_y_;

  // These are static, we are guaranteed the Draw() is called sequentially.
  static bool lists_created_;
  static std::unique_ptr<Model3DS> model_;
  static GLuint robot_dl_;
};

#endif
