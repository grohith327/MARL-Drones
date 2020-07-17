#include "bicycle.h"

#include <cmath>

#include "util/utils.h"

namespace {
constexpr double kMaxV = 1.;
constexpr double kMaxPhi = 3.14 / 15.0;

// Drawing constants.
constexpr char kModelPath[] = "./models/car.3ds";
constexpr double kRearDistance = 0.047;
constexpr double kFrontDistance = 0.202 - kRearDistance;
constexpr double kTopDistance = 0.076;
constexpr double kSideDistance = 0.045;

// Control constants.
constexpr double kWheelbase = 0.12;
constexpr double kDistanceThreshold = 0.1;
constexpr double kGain = 2.0;
}  // namespace

// static
bool Bicycle::lists_created_ = false;
// static
std::unique_ptr<Model3DS> Bicycle::model_;
// static
GLuint Bicycle::robot_dl_ = 0;

Bicycle::Bicycle() : v_(0.0), phi_(0.0) {}

bool Bicycle::Init() {
  return Robot::Init();
}

void Bicycle::Step(double t, double dt) {
  // Update position based on previous control inputs.
  // Based on simple euler integration.
  x_ += cos(yaw_) * v_ * dt;
  y_ += sin(yaw_) * v_ * dt;
  yaw_ = NormalizeAngle(yaw_ + tan(phi_) * v_ / kWheelbase * dt);
  Robot::Step(t, dt);

  // Go to the goal if desired.
  if (GoalBased()) {
    double v = 0.0;
    double phi = 0.0;
    double dx = goal_x_ - x_;
    double dy = goal_y_ - y_;
    double e = sqrtf(dx * dx + dy * dy);
    if (e > kDistanceThreshold) {
      // Go to goal point.
      // Not supported for now.
    }
    // Sets controls.
    SetControlInputs(v, phi);
  }
}

void Bicycle::SetControlInputs(double v, double phi) {
  v_ = std::max(std::min(v, kMaxV), -kMaxV);
  phi_ = std::max(std::min(phi, kMaxPhi), -kMaxPhi);
}

void Bicycle::SetGoal(double x, double y) {
  goal_x_ = x;
  goal_y_ = y;
}

void Bicycle::Draw(VisualizerWindow* window) {
  // Reload compiled lists if needed.
  if (!lists_created_ || window->HasContextChanged()) {
    if (lists_created_ && robot_dl_ != 0) {
      glDeleteLists(robot_dl_, 1);
    }
    model_.reset(new Model3DS());
    model_->Load(kModelPath);
    robot_dl_ = glGenLists(1);
    glNewList(robot_dl_, GL_COMPILE);
    model_->Draw();
    glEndList();
    lists_created_ = true;
  }

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(x_, 0.0, -y_);
  glRotatef(yaw_ / M_PI * 180.0 - 90.0, 0.0, 1.0, 0.0);
  glColor3f(0.5 * std::get<0>(color_), 0.5 * std::get<1>(color_), 0.5 * std::get<2>(color_));

  double xv, yv, zv;
  window->GetCenterOfAttention(&xv, &yv, &zv);
  float dist = sqrtf((x_ - xv) * (x_ - xv) + (y_ - zv) * (y_ - zv));
  if (dist < 10.0 && !window->IsFastModeEnabled()) {
    glCallList(robot_dl_);
  } else {
    // Just draw outer vehicle dimensions.
    glPushMatrix();
    glTranslatef(0.0, kTopDistance / 2.0,  + ((kFrontDistance + kRearDistance) / 2.0 - kFrontDistance));
    glScalef(2.0 * kSideDistance, kTopDistance, kFrontDistance + kRearDistance);
    glutWireCube(1.0);
    glPopMatrix();
  }
  glPopMatrix();
}
