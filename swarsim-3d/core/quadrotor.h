#ifndef _QUADROTOR_H
#define _QUADROTOR_H

#include <memory>

#include "display/model_3ds.h"
#include "display/window.h"
#include "robot.h"

class Quadrotor : public Robot {
 public:
  Quadrotor();

  // Initializes the robots.
  bool Init() final;

  // Executes a step.
  void Step(double t, double dt) final;

  // Draw the robot.
  void Draw(VisualizerWindow* window) override;

  bool IsGround() const final { return false; };
  bool IsFlying() const final { return true; };

 protected:
  // Sets the control inputs for the next time step.
  void SetControlInputs(double up_force, double roll_torque, double pitch_torque, double yaw_torque);

  // Sets the goal position if GoalBased returns true.
  void SetGoal(double x, double y, double z);

 protected:
  // Internal state.
  double xdot_;
  double ydot_;
  double zdot_;
  double rolldot_;
  double pitchdot_;
  double yawdot_;


 private:
  void ComputeDesiredRPM();

  // Control inputs.
  double up_force_;
  double roll_torque_;
  double pitch_torque_;
  double yaw_torque_;

  // The above control inputs, translate to desired rotor speeds.
  double w1_desired_;
  double w2_desired_;
  double w3_desired_;
  double w4_desired_;

  // Goal position.
  double goal_x_;
  double goal_y_;
  double goal_z_;

  // Additional internal state.
  double I_[3];
  double w1_;
  double w2_;
  double w3_;
  double w4_;
  double r1a_;
  double r2a_;
  double r3a_;
  double r4a_;
  double xbdes_[3];
  double ybdes_[3];
  double zbdes_[3];

  // These are static, we are guaranteed the Draw() is called sequentially.
  static bool lists_created_;
  static std::unique_ptr<Model3DS> robot_model_;
  static std::unique_ptr<Model3DS> rotor_model_;
  static GLuint robot_dl_;
  static GLuint rotor_dl_;
};

#endif
