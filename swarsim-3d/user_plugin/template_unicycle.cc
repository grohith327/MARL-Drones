#include "template_unicycle.h"

// Your controller can accept commandline flags.
// Feel free to add new flags here. E.g.:
// DEFINE_bool(my_boolean_flag, true, "description");

bool TemplateUnicycle::Initialize() {
  // This function should true upon success.
  // Initialize any variables you need here.
  return true;
}

void TemplateUnicycle::Execute(double t, double dt) {
  // Depending on whether GoalBased() - declared in the header
  // file return, this function should either call SetGoal(x, y) or
  // SetControlInputs(u, w).

  // Let's have the robot turn in circles. Note that since the
  // ground robot is a Khepera III robot, it's limited to 0.5 m/s
  // in forward velocity.
  SetControlInputs(0.2 /* m/s */, 1 /* rad/s */);

  // Controllers can use additional functions such as:
  // neighbors(), x(), y(), yaw().
}

void TemplateUnicycle::Draw(VisualizerWindow* window) {
  Unicycle::Draw(window);

  // Feel free to draw any additional information on screen.
  // Any OpenGL functions should work.
  // If you do not wish to draw anything, feel free to delete
  // this function and its declaration in the header file.
}
