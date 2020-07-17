#include "unicycle.h"

#include <cmath>

#include "util/utils.h"

namespace {
constexpr double kMaxU = 0.5;
constexpr double kMaxW = 0.5 / (0.021 * 0.045); // = (1.0 - max_u) / (wheel_radius * axle_length / 2)

// Drawing constants.
constexpr char kModelPath[] = "./models/k3.3ds";
constexpr double kFrontDistance = 0.07;
constexpr double kRearDistance = 0.045;
constexpr double kTopDistance = 0.07;
constexpr double kSideDistance = 0.065;

// Control constants.
constexpr double kGainU = 2.0;
constexpr double kGainW = 4.0;
constexpr double kDistanceThreshold = 0.1;
constexpr double kSafetyDistance = 0.3;
constexpr double kActiveRegion = 0.6;
constexpr double kGainAvoidanceU = 6.0;
constexpr double kGainAvoidanceW = 12.0;
}  // namespace

// static
bool Unicycle::lists_created_ = false;
// static
std::unique_ptr<Model3DS> Unicycle::model_;
// static
GLuint Unicycle::robot_dl_ = 0;

Unicycle::Unicycle() : u_(0.0), w_(0.0) {}

bool Unicycle::Init() {
  return Robot::Init();
}

void Unicycle::Step(double t, double dt) {
  static double avoidance_factor = 1.0 / sqrt(kSafetyDistance) - 1.0 / sqrt(kActiveRegion);

  // Update position based on previous control inputs.
  // Based on simple euler integration.
  x_ += cos(yaw_) * u_ * dt;
  y_ += sin(yaw_) * u_ * dt;
  yaw_ = NormalizeAngle(yaw_ + w_ * dt);
  Robot::Step(t, dt);

  // Go to the goal if desired.
  if (GoalBased()) {
    double u = 0.0;
    double w = 0.0;
    double dx = goal_x_ - x_;
    double dy = goal_y_ - y_;
    double e = sqrtf(dx * dx + dy * dy);
    if (e > kDistanceThreshold) {
      // Go to goal point.
      double a = NormalizeAngle(atan2(dy, dx) - yaw_);
      u = kGainU * e * cosf(a);
      w = kGainW * sinf(a);
    }
    // Correct inputs for neighbor avoidance.
    for (const Robot* neighbor : neighbors()) {
      if (neighbor->IsFlying()) continue;
      dx = neighbor->x() - x_;
      dy = neighbor->y() - y_;
      e = sqrt(dx * dx + dy * dy);
      double a = NormalizeAngle(atan2(dy, dx) - yaw_);
      if (e < kActiveRegion) {
        e = std::max(e, 0.05);
        double f = avoidance_factor * (1.0 / sqrtf(e) - 1.0 / sqrt(kActiveRegion));
        u -= kGainAvoidanceU * e * cosf(a) * f;
        w -= kGainAvoidanceW * sinf(a) * f;
      }
    }
    // Sets controls.
    SetControlInputs(u, w);
  }
}

void Unicycle::SetControlInputs(double u, double w) {
  u_ = std::max(std::min(u, kMaxU), -kMaxU);
  w_ = std::max(std::min(w, kMaxW), -kMaxW);
}

void Unicycle::SetGoal(double x, double y) {
  goal_x_ = x;
  goal_y_ = y;
}

void Unicycle::Draw(VisualizerWindow* window) {
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
