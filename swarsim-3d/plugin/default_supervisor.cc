#include "default_supervisor.h"

#ifdef MAC
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <math.h>

#include "gflags/gflags.h"
#include "util/utils.h"

DEFINE_int32(default_supervisor_nrobots, 1, "Number of robots to initialize.");

namespace {
// Control constants.
constexpr double kArenaSize = 4.0;
constexpr double kCameraDuration = 5.0;
}  // namespace

// bool DefaultSupervisor::Initialize() {
//   for (int i = 0; i < FLAGS_default_supervisor_nrobots; ++i) {
//     Robot* robot;
//     if (i % 2 == 0) {
//       robot = CreateRobot("DefaultGroundRobot");
//       // Random positions.
//       robot->SetPosition(RandomUniform() * kArenaSize - kArenaSize / 2.0,
//                          RandomUniform() * kArenaSize - kArenaSize / 2.0,
//                          RandomUniform() * M_PI * 2.0);
//     } else  {
//       robot = CreateRobot("DefaultFlyingRobot");
//       // Random positions.
//       robot->SetPosition(RandomUniform() * kArenaSize - kArenaSize / 2.0,
//                          RandomUniform() * kArenaSize - kArenaSize / 2.0,
//                          RandomUniform() * kArenaSize / 2.0,
//                          0.0,
//                          0.0,
//                          RandomUniform() * M_PI * 2.0);
//     }
//     // Assign type. Robots of different types appear in different colors.
//     robot->SetType(i);
//   }
//   return true;
// }

bool DefaultSupervisor::Initialize() {
  for (int i = 0; i < FLAGS_default_supervisor_nrobots; ++i) {
    Robot* robot;
    if (i % 2 == 0) {
      robot = CreateRobot("DefaultQuadrotor");
      robot->SetPosition(RandomUniform() * kArenaSize - kArenaSize / 2.0,
                         RandomUniform() * kArenaSize - kArenaSize / 2.0,
                         RandomUniform() * M_PI * 2.0);
      // robot->SetPosition(0.0,
      //                    0.0,
      //                    RandomUniform() * M_PI * 2.0);
      
    } else  {
      robot = CreateRobot("DefaultQuadrotor");
      robot->SetPosition(RandomUniform() * kArenaSize - kArenaSize / 2.0,
                         RandomUniform() * kArenaSize - kArenaSize / 2.0,
                         RandomUniform() * kArenaSize / 2.0,
                         0.0,
                         0.0,
                         RandomUniform() * M_PI * 2.0);

      // robot->SetPosition(0.0,
      //                    2.0,
      //                    0.0,
      //                    0.0,
      //                    0.0,
      //                    RandomUniform() * M_PI * 2.0);
      // std::cout<<"Robot Type: "<<robot2->type()<<std::endl;
    }
    // Assign type. Robots of different types appear in different colors.
    robot->SetType(i);
  }
  return true;
}

void DefaultSupervisor::Draw(double t, VisualizerWindow* window) {
  // Draw a small rectangle on the floor.
  glColor3f(0.3, 0.3, 0.3);
  glBegin(GL_QUADS);
  glLineWidth(1.0);
  glVertex3f(-kArenaSize / 2.0, -0.01, -kArenaSize / 2.0);
  glVertex3f(-kArenaSize / 2.0, -0.01, kArenaSize / 2.0);
  glVertex3f(kArenaSize / 2.0, -0.01, kArenaSize / 2.0);
  glVertex3f(kArenaSize / 2.0, -0.01, -kArenaSize / 2.0);
  glEnd();
}

// void DefaultSupervisor::Update(double t, double dt)
// {
  
// }
