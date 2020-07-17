#include "template_supervisor.h"

#include <math.h>

#include "gflags/gflags.h"
#include "util/utils.h"

// Your supervisor can accept commandline flags.
// Feel free to add new flags here. E.g.:
DEFINE_int32(template_supervisor_nrobots, 4, "Number of robots to initialize.");

namespace {
// Control constants.
constexpr double kArenaSize = 3.0;
}  // namespace

bool TemplateSupervisor::Initialize() {
  // Initialize all robots.
  for (int i = 0; i < FLAGS_template_supervisor_nrobots; ++i) {
    Robot* robot;
    robot = CreateRobot("TemplateGroundRobot");
    // Random positions.
    robot->SetPosition(RandomUniform() * kArenaSize - kArenaSize / 2.0,
                       RandomUniform() * kArenaSize - kArenaSize / 2.0,
                       RandomUniform() * M_PI * 2.0);
    // Assign type. Robots of different types appear in different colors.
    robot->SetType(i);
  }

  // Return true in case of success.
  return true;
}

void TemplateSupervisor::Draw(double t, VisualizerWindow* window) {
  // Feel free to draw any additional information on screen.
  // Any OpenGL functions should work.
  // If you do not wish to draw anything, feel free to delete
  // this function and its declaration in the header file.
}

void TemplateSupervisor::Update(double t, double dt) {
  // Update any internal variables based on the time.
}
