#include "quadrotor.h"

#include <cmath>
#include <iostream>

#include "gflags/gflags.h"
#include "util/utils.h"

DEFINE_bool(quadrotor_axes, false, "Draw target axes if desired.");

namespace {
// Drawing constants.
constexpr char kQuadModelPath[] = "./models/quadrotor.3ds";
constexpr char kRotorModelPath[] = "./models/rotor.3ds";
constexpr float kHeightArm = 0.1;
constexpr float kLengthArm = 0.5;
constexpr float kWidthArm = 0.1;

// Control constants.
constexpr double kEpsilon = 0.00001;
constexpr double kSafetyZDistance = 0.1;
constexpr double kActiveXYRegion = 1.0;
constexpr double kActiveZRegion = 0.5;
constexpr double kKP = 1.0;
constexpr double kKV = 2.0;
constexpr double kKR = 10.0;
constexpr double kKW = 5.0;
constexpr double kKRYaw = 0.1;
constexpr double kKWYaw = 0.3;
constexpr double kMaxGoalDistance = 1.2;

// Gravity.
constexpr double kGravity = 9.81;

// Constants for typical quadrotor.
constexpr double kMass = 0.5;
constexpr double kKF = 6.11e-8;
constexpr double kKM = 1.5e-9;
constexpr double kHalfArmLength = 0.25;
constexpr double kMaxRotorSpeed = 7800.0;
constexpr double kMinRotorSpeed = 1200.0;
constexpr double kRotorMass = 0.025;
constexpr double kInertiaXY = 0.2;
constexpr double kInertiaZ =   0.2;
}  // namespace

// static
bool Quadrotor::lists_created_ = false;
// static
std::unique_ptr<Model3DS> Quadrotor::robot_model_;
// static
std::unique_ptr<Model3DS> Quadrotor::rotor_model_;
// static
GLuint Quadrotor::robot_dl_ = 0;
// static
GLuint Quadrotor::rotor_dl_ = 0;

Quadrotor::Quadrotor() {
  xdot_ = 0.0;
  ydot_ = 0.0;
  zdot_ = 0.0;
  rolldot_ = 0.0;
  pitchdot_ = 0.0;
  yawdot_ = 0.0;

  up_force_ = kGravity * kMass;
  roll_torque_ = 0.0;
  pitch_torque_ = 0.0;
  yaw_torque_ = 0.0;

  r1a_ = 0.0;
  r2a_ = 0.0;
  r3a_ = 0.0;
  r4a_ = 0.0;
  ComputeDesiredRPM();
  w1_ = w1_desired_;
  w2_ = w2_desired_;
  w3_ = w3_desired_;
  w4_ = w4_desired_;

  I_[0] = kInertiaXY;
  I_[1] = I_[0];
  I_[2] = kInertiaZ;

  xbdes_[0] = 0.0; xbdes_[1] = 0.0; xbdes_[2] = 0.0;
  ybdes_[0] = 0.0; ybdes_[1] = 0.0; ybdes_[2] = 0.0;
  zbdes_[0] = 0.0; zbdes_[1] = 0.0; zbdes_[2] = 0.0;
}

bool Quadrotor::Init() {
  return Robot::Init();
}

void Quadrotor::Step(double t, double dt) {
  static double avoidance_factor_z = 1.0 / sqrt(kSafetyZDistance) - 1.0 / sqrt(kActiveZRegion);

  // Update position based on previous control inputs.
  // Based on Runge - Kutta integration.
  double kw1[4], kw2[4], kw3[4], kw4[4];
  kw1[0] = 20.0 * (w1_desired_ - w1_) * dt;
  kw1[1] = 20.0 * (w1_desired_ - (w1_ + 0.5 * kw1[0])) * dt;
  kw1[2] = 20.0 * (w1_desired_ - (w1_ + 0.5 * kw1[1])) * dt;
  kw1[3] = 20.0 * (w1_desired_ - (w1_ + kw1[2])) * dt;
  kw2[0] = 20.0 * (w2_desired_ - w2_) * dt;
  kw2[1] = 20.0 * (w2_desired_ - (w2_ + 0.5 * kw2[0])) * dt;
  kw2[2] = 20.0 * (w2_desired_ - (w2_ + 0.5 * kw2[1])) * dt;
  kw2[3] = 20.0 * (w2_desired_ - (w2_ + kw2[2])) * dt;
  kw3[0] = 20.0 * (w3_desired_ - w3_) * dt;
  kw3[1] = 20.0 * (w3_desired_ - (w3_ + 0.5 * kw3[0])) * dt;
  kw3[2] = 20.0 * (w3_desired_ - (w3_ + 0.5 * kw3[1])) * dt;
  kw3[3] = 20.0 * (w3_desired_ - (w3_ + kw3[2])) * dt;
  kw4[0] = 20.0 * (w4_desired_ - w4_) * dt;
  kw4[1] = 20.0 * (w4_desired_ - (w4_ + 0.5 * kw4[0])) * dt;
  kw4[2] = 20.0 * (w4_desired_ - (w4_ + 0.5 * kw4[1])) * dt;
  kw4[3] = 20.0 * (w4_desired_ - (w4_ + kw4[2])) * dt;

  double kx[4], ky[4], kz[4], kxdot[4], kydot[4], kzdot[4];
  double kroll[4], kpitch[4], kyaw[4], krolldot[4], kpitchdot[4], kyawdot[4];

  // Compute k1
  float c1 = cos(roll_);
  float c2 = cos(pitch_);
  float c3 = cos(yaw_);
  float s1 = sin(roll_);
  float s2 = sin(pitch_);
  float s3 = sin(yaw_);
  double zx = c1 * c3 * s2 + s1 * s3;
  double zy = -c3 * s1 + c1 * s2 * s3;
  double zz = c1 * c2;
  double u1 = (w1_ * w1_ + w2_ * w2_ + w3_ * w3_ + w4_ * w4_) * kKF;
  double ax = u1 * zx;
  double ay = u1 * zy;
  double az = u1 * zz - kMass * kGravity;
  ax /= kMass;
  ay /= kMass;
  az /= kMass;
  kxdot[0] = ax * dt;
  kydot[0] = ay * dt;
  kzdot[0] = az * dt;
  kx[0] = xdot_ * dt;
  ky[0] = ydot_ * dt;
  kz[0] = zdot_ * dt;
  double u2 = (w2_ * w2_ - w4_ * w4_) * kKF * kHalfArmLength;
  double u3 = (-w1_ * w1_ + w3_ * w3_) * kKF * kHalfArmLength;
  double u4 = (w1_ * w1_ - w2_ * w2_ + w3_ * w3_ - w4_ * w4_) * kKM;
  double rollddot  = ((pitchdot_ * yawdot_) * (I_[1] - I_[2]) + u2) / (I_[0]);
  double pitchddot = ((rolldot_ * yawdot_) * (I_[2] - I_[0]) + u3) / (I_[1]);
  double yawddot   = ((pitchdot_ * rolldot_) * (I_[1] - I_[0]) + u4) / (I_[2]);
  krolldot[0] = rollddot * dt;
  kpitchdot[0] = pitchddot * dt;
  kyawdot[0] = yawddot * dt;
  kroll[0] = rolldot_ * dt;
  kpitch[0] = pitchdot_ * dt;
  kyaw[0] = yawdot_ * dt;

  // Compute k2
  c1 = cos(roll_ + 0.5 * kroll[0]);
  c2 = cos(pitch_ + 0.5 * kpitch[0]);
  c3 = cos(yaw_ + 0.5 * kyaw[0]);
  s1 = sin(roll_ + 0.5 * kroll[0]);
  s2 = sin(pitch_ + 0.5 * kpitch[0]);
  s3 = sin(yaw_ + 0.5 * kyaw[0]);
  zx = c1 * c3 * s2 + s1 * s3;
  zy = -c3 * s1 + c1 * s2 * s3;
  zz = c1 * c2;
  u1 = ((w1_ + 0.5 * kw1[0]) * (w1_ + 0.5 * kw1[0]) + (w2_ + 0.5 * kw2[0]) * (w2_ + 0.5 * kw2[0]) + (w3_ + 0.5 * kw3[0]) * (w3_ + 0.5 * kw3[0]) + (w4_ + 0.5 * kw4[0]) * (w4_ + 0.5 * kw4[0])) * kKF;
  ax = u1 * zx;
  ay = u1 * zy;
  az = u1 * zz - kMass * kGravity;
  ax /= kMass;
  ay /= kMass;
  az /= kMass;
  kxdot[1] = ax * dt;
  kydot[1] = ay * dt;
  kzdot[1] = az * dt;
  kx[1] = (xdot_ + 0.5 * kxdot[0]) * dt;
  ky[1] = (ydot_ + 0.5 * kydot[0]) * dt;
  kz[1] = (zdot_ + 0.5 * kzdot[0]) * dt;
  u2 = ((w2_ + 0.5 * kw2[0]) * (w2_ + 0.5 * kw2[0]) - (w4_ + 0.5 * kw4[0]) * (w4_ + 0.5 * kw4[0])) * kKF * kHalfArmLength;
  u3 = (-(w1_ + 0.5 * kw1[0]) * (w1_ + 0.5 * kw1[0]) + (w3_ + 0.5 * kw3[0]) * (w3_ + 0.5 * kw3[0])) * kKF * kHalfArmLength;
  u4 = ((w1_ + 0.5 * kw1[0]) * (w1_ + 0.5 * kw1[0]) - (w2_ + 0.5 * kw2[0]) * (w2_ + 0.5 * kw2[0]) + (w3_ + 0.5 * kw3[0]) * (w3_ + 0.5 * kw3[0]) - (w4_ + 0.5 * kw4[0]) * (w4_ + 0.5 * kw4[0])) * kKM;
  rollddot  = (((pitchdot_ + 0.5 * kpitchdot[0]) * (yawdot_ + 0.5 * kyawdot[0])) * (I_[1] - I_[2]) + u2) / (I_[0]);
  pitchddot = (((rolldot_ + 0.5 * krolldot[0]) * (yawdot_ + 0.5 * kyawdot[0])) * (I_[2] - I_[0]) + u3) / (I_[1]);
  yawddot   = (((pitchdot_ + 0.5 * kpitchdot[0]) * (rolldot_ + 0.5 * krolldot[0])) * (I_[1] - I_[0]) + u4) / (I_[2]);
  krolldot[1] = rollddot * dt;
  kpitchdot[1] = pitchddot * dt;
  kyawdot[1] = yawddot * dt;
  kroll[1] = (rolldot_ + 0.5 * krolldot[0]) * dt;
  kpitch[1] = (pitchdot_ + 0.5 * kpitchdot[0]) * dt;
  kyaw[1] = (yawdot_ + 0.5 * kyawdot[0]) * dt;

  // Compute k3
  c1 = cos(roll_ + 0.5 * kroll[1]);
  c2 = cos(pitch_ + 0.5 * kpitch[1]);
  c3 = cos(yaw_ + 0.5 * kyaw[1]);
  s1 = sin(roll_ + 0.5 * kroll[1]);
  s2 = sin(pitch_ + 0.5 * kpitch[1]);
  s3 = sin(yaw_ + 0.5 * kyaw[1]);
  zx = c1 * c3 * s2 + s1 * s3;
  zy = -c3 * s1 + c1 * s2 * s3;
  zz = c1 * c2;
  u1 = ((w1_ + 0.5 * kw1[1]) * (w1_ + 0.5 * kw1[1]) + (w2_ + 0.5 * kw2[1]) * (w2_ + 0.5 * kw2[1]) + (w3_ + 0.5 * kw3[1]) * (w3_ + 0.5 * kw3[1]) + (w4_ + 0.5 * kw4[1]) * (w4_ + 0.5 * kw4[1])) * kKF;
  ax = u1 * zx;
  ay = u1 * zy;
  az = u1 * zz - kMass * kGravity;
  ax /= kMass;
  ay /= kMass;
  az /= kMass;
  kxdot[2] = ax * dt;
  kydot[2] = ay * dt;
  kzdot[2] = az * dt;
  kx[2] = (xdot_ + 0.5 * kxdot[1]) * dt;
  ky[2] = (ydot_ + 0.5 * kydot[1]) * dt;
  kz[2] = (zdot_ + 0.5 * kzdot[1]) * dt;
  u2 = ((w2_ + 0.5 * kw2[1]) * (w2_ + 0.5 * kw2[1]) - (w4_ + 0.5 * kw4[1]) * (w4_ + 0.5 * kw4[1])) * kKF * kHalfArmLength;
  u3 = (-(w1_ + 0.5 * kw1[1]) * (w1_ + 0.5 * kw1[1]) + (w3_ + 0.5 * kw3[1]) * (w3_ + 0.5 * kw3[1])) * kKF * kHalfArmLength;
  u4 = ((w1_ + 0.5 * kw1[1]) * (w1_ + 0.5 * kw1[1]) - (w2_ + 0.5 * kw2[1]) * (w2_ + 0.5 * kw2[1]) + (w3_ + 0.5 * kw3[1]) * (w3_ + 0.5 * kw3[1]) - (w4_ + 0.5 * kw4[1]) * (w4_ + 0.5 * kw4[1])) * kKM;
  rollddot  = (((pitchdot_ + 0.5 * kpitchdot[1]) * (yawdot_ + 0.5 * kyawdot[1])) * (I_[1] - I_[2]) + u2) / (I_[0]);
  pitchddot = (((rolldot_ + 0.5 * krolldot[1]) * (yawdot_ + 0.5 * kyawdot[1])) * (I_[2] - I_[0]) + u3) / (I_[1]);
  yawddot   = (((pitchdot_ + 0.5 * kpitchdot[1]) * (rolldot_ + 0.5 * krolldot[1])) * (I_[1] - I_[0]) + u4) / (I_[2]);
  krolldot[2] = rollddot * dt;
  kpitchdot[2] = pitchddot * dt;
  kyawdot[2] = yawddot * dt;
  kroll[2] = (rolldot_ + 0.5 * krolldot[1]) * dt;
  kpitch[2] = (pitchdot_ + 0.5 * kpitchdot[1]) * dt;
  kyaw[2] = (yawdot_ + 0.5 * kyawdot[1]) * dt;

  // Compute k4
  c1 = cos(roll_ + kroll[2]);
  c2 = cos(pitch_ + kpitch[2]);
  c3 = cos(yaw_ + kyaw[2]);
  s1 = sin(roll_ + kroll[2]);
  s2 = sin(pitch_ + kpitch[2]);
  s3 = sin(yaw_ + kyaw[2]);
  zx = c1 * c3 * s2 + s1 * s3;
  zy = -c3 * s1 + c1 * s2 * s3;
  zz = c1 * c2;
  u1 = ((w1_ + kw1[2]) * (w1_ + kw1[2]) + (w2_ + kw2[2]) * (w2_ + kw2[2]) + (w3_ + kw3[2]) * (w3_ + kw3[2]) + (w4_ + kw4[2]) * (w4_ + kw4[2])) * kKF;
  ax = u1 * zx;
  ay = u1 * zy;
  az = u1 * zz - kMass * kGravity;
  ax /= kMass;
  ay /= kMass;
  az /= kMass;
  kxdot[3] = ax * dt;
  kydot[3] = ay * dt;
  kzdot[3] = az * dt;
  kx[3] = (xdot_ + kxdot[2]) * dt;
  ky[3] = (ydot_ + kydot[2]) * dt;
  kz[3] = (zdot_ + kzdot[2]) * dt;
  u2 = ((w2_ + kw2[2]) * (w2_ + kw2[2]) - (w4_ + kw4[2]) * (w4_ + kw4[2])) * kKF * kHalfArmLength;
  u3 = (-(w1_ + kw1[2]) * (w1_ + kw1[2]) + (w3_ + kw3[2]) * (w3_ + kw3[2])) * kKF * kHalfArmLength;
  u4 = ((w1_ + kw1[2]) * (w1_ + kw1[2]) - (w2_ + kw2[2]) * (w2_ + kw2[2]) + (w3_ + kw3[2]) * (w3_ + kw3[2]) - (w4_ + kw4[2]) * (w4_ + kw4[2])) * kKM;
  rollddot  = (((pitchdot_ + kpitchdot[2]) * (yawdot_ + kyawdot[2])) * (I_[1] - I_[2]) + u2) / (I_[0]);
  pitchddot = (((rolldot_ + krolldot[2]) * (yawdot_ + kyawdot[2])) * (I_[2] - I_[0]) + u3) / (I_[1]);
  yawddot   = (((pitchdot_ + kpitchdot[2]) * (rolldot_ + krolldot[2])) * (I_[1] - I_[0]) + u4) / (I_[2]);
  krolldot[3] = rollddot * dt;
  kpitchdot[3] = pitchddot * dt;
  kyawdot[3] = yawddot * dt;
  kroll[3] = (rolldot_ + krolldot[2]) * dt;
  kpitch[3] = (pitchdot_ + kpitchdot[2]) * dt;
  kyaw[3] = (yawdot_ + kyawdot[2]) * dt;

  xdot_ += (kxdot[0] + 2.0 * kxdot[1] + 2.0 * kxdot[2] + kxdot[3]) / 6.0;
  ydot_ += (kydot[0] + 2.0 * kydot[1] + 2.0 * kydot[2] + kydot[3]) / 6.0;
  zdot_ += (kzdot[0] + 2.0 * kzdot[1] + 2.0 * kzdot[2] + kzdot[3]) / 6.0;
  x_ += (kx[0] + 2.0 * kx[1] + 2.0 * kx[2] + kx[3]) / 6.0;
  y_ += (ky[0] + 2.0 * ky[1] + 2.0 * ky[2] + ky[3]) / 6.0;
  z_ += (kz[0] + 2.0 * kz[1] + 2.0 * kz[2] + kz[3]) / 6.0;

  rolldot_ += (krolldot[0] + 2.0 * krolldot[1] + 2.0 * krolldot[2] + krolldot[3]) / 6.0;
  pitchdot_ += (kpitchdot[0] + 2.0 * kpitchdot[1] + 2.0 * kpitchdot[2] + kpitchdot[3]) / 6.0;
  yawdot_ += (kyawdot[0] + 2.0 * kyawdot[1] + 2.0 * kyawdot[2] + kyawdot[3]) / 6.0;
  roll_ += (kroll[0] + 2.0 * kroll[1] + 2.0 * kroll[2] + kroll[3]) / 6.0;
  pitch_ += (kpitch[0] + 2.0 * kpitch[1] + 2.0 * kpitch[2] + kpitch[3]) / 6.0;
  yaw_ += (kyaw[0] + 2.0 * kyaw[1] + 2.0 * kyaw[2] + kyaw[3]) / 6.0;

  w1_ += (kw1[0] + 2.0 * kw1[1] + 2.0 * kw1[2] + kw1[3]) / 6.0;
  w2_ += (kw2[0] + 2.0 * kw2[1] + 2.0 * kw2[2] + kw2[3]) / 6.0;
  w3_ += (kw3[0] + 2.0 * kw3[1] + 2.0 * kw3[2] + kw3[3]) / 6.0;
  w4_ += (kw4[0] + 2.0 * kw4[1] + 2.0 * kw4[2] + kw4[3]) / 6.0;

  // Normalize angles
  roll_ = NormalizeAngle(roll_);
  pitch_ = NormalizeAngle(pitch_);
  yaw_ = NormalizeAngle(yaw_);

  // Just for display so Euler is good enough
  r1a_ += w1_ * dt * (2.0 * M_PI) / 60.0;
  r2a_ += w2_ * dt * (2.0 * M_PI) / 60.0;
  r3a_ += w3_ * dt * (2.0 * M_PI) / 60.0;
  r4a_ += w4_ * dt * (2.0 * M_PI) / 60.0;
  r1a_ = NormalizeAngle(r1a_);
  r2a_ = NormalizeAngle(r2a_);
  r3a_ = NormalizeAngle(r3a_);
  r4a_ = NormalizeAngle(r4a_);

  Robot::Step(t, dt);

  // Go to the goal if desired (compute desired forces).
  if (GoalBased()) {
    double yawdes = yaw_;
    // Desired delta.
    double xdes = 0.0;
    double ydes = 0.0;
    double zdes = 0.0;

    // Go to the goal.
    double dx = goal_x_ - x_;
    double dy = goal_y_ - y_;
    double dz = goal_z_ - z_;
    xdes += dx;
    ydes += dy;
    zdes += dz;

    // Avoid flying neighbors.
    for (const Robot* neighbor : neighbors()) {
      if (neighbor->IsGround()) continue;
      dx = neighbor->x() - x_;
      dy = neighbor->y() - y_;
      dz = neighbor->z() - y_;
      double xy_e = sqrt(dx * dx + dy * dy);
      double z_e = std::abs(dz);
      if (xy_e < kActiveXYRegion && z_e < kActiveZRegion) {
        z_e = std::max(std::abs(dz), 0.05);
        double f = avoidance_factor_z * (1.0 / sqrt(z_e) - 1.0 / sqrt(kActiveZRegion));
        zdes -= dz * f;
      }
    }
    // Get repulsed by ground
    if (z_ < kActiveZRegion) {
      double e = std::max(z_, 0.05);
      double f = avoidance_factor_z * (1.0 / sqrtf(e) - 1.0 / sqrt(kActiveZRegion));
      zdes += e * f;
    }

    double norm = sqrt(xdes * xdes + ydes * ydes + zdes * zdes);
    if (norm > kMaxGoalDistance) {
      xdes *= kMaxGoalDistance / norm;
      ydes *= kMaxGoalDistance / norm;
      zdes *= kMaxGoalDistance / norm;
    }

    double Fdes[3]; // desired force
    Fdes[0] = kKP * xdes - kKV * (xdot_);
    Fdes[1] = kKP * ydes - kKV * (ydot_);
    Fdes[2] = kKP * zdes - kKV * (zdot_) + kMass * kGravity;

    // Body axis
    float c1 = cos(roll_);
    float c2 = cos(pitch_);
    float c3 = cos(yaw_);
    float s1 = sin(roll_);
    float s2 = sin(pitch_);
    float s3 = sin(yaw_);
    double xx = c2 * c3;
    double xy = c2 * s3;
    double xz = -s2;
    double yx = c3 * s1 * s2 - c1 * s3;
    double yy = c1 * c3 + s1 * s2 * s3;
    double yz = c2 * s1;
    double zx = c1 * c3 * s2 + s1 * s3;
    double zy = -c3 * s1 + c1 * s2 * s3;
    double zz = c1 * c2;

    up_force_ = Fdes[0] * zx + Fdes[1] * zy + Fdes[2] * zz;

    // Compute axis desired.

    // Z axis.
    double n = sqrt(Fdes[0] * Fdes[0] + Fdes[1] * Fdes[1] + Fdes[2] * Fdes[2]);
    if (std::abs(n) > kEpsilon) {
      zbdes_[0] = Fdes[0] / n; zbdes_[1] = Fdes[1] / n; zbdes_[2] = Fdes[2] / n;
      // Y axis.
      double xcdes[3] = {cos(yawdes),sin(yawdes),0.0};
      ybdes_[0] = zbdes_[1] * xcdes[2] - zbdes_[2] * xcdes[1];
      ybdes_[1] = zbdes_[2] * xcdes[0] - zbdes_[0] * xcdes[2];
      ybdes_[2] = zbdes_[0] * xcdes[1] - zbdes_[1] * xcdes[0];
      n = sqrt(ybdes_[0] * ybdes_[0] + ybdes_[1] * ybdes_[1] + ybdes_[2] * ybdes_[2]);
      if (fabs(n) < kEpsilon) {
        ybdes_[0] = -sin(yaw_); ybdes_[1] = cos(yaw_); ybdes_[2] = 0.0; // Not sure about this
      } else {
        ybdes_[0] /= n; ybdes_[1] /= n; ybdes_[2] /= n;
      }
      // X axis.
      xbdes_[0] = ybdes_[1] * zbdes_[2] - ybdes_[2] * zbdes_[1];
      xbdes_[1] = ybdes_[2] * zbdes_[0] - ybdes_[0] * zbdes_[2];
      xbdes_[2] = ybdes_[0] * zbdes_[1] - ybdes_[1] * zbdes_[0];

      // R^T x Rdes.
      double R[3][3];
      R[0][0] = xx * xbdes_[0] + xy * xbdes_[1] + xz * xbdes_[2];
      R[0][1] = xx * ybdes_[0] + xy * ybdes_[1] + xz * ybdes_[2];
      R[0][2] = xx * zbdes_[0] + xy * zbdes_[1] + xz * zbdes_[2];
      R[1][0] = yx * xbdes_[0] + yy * xbdes_[1] + yz * xbdes_[2];
      R[1][1] = yx * ybdes_[0] + yy * ybdes_[1] + yz * ybdes_[2];
      R[1][2] = yx * zbdes_[0] + yy * zbdes_[1] + yz * zbdes_[2];
      R[2][0] = zx * xbdes_[0] + zy * xbdes_[1] + zz * xbdes_[2];
      R[2][1] = zx * ybdes_[0] + zy * ybdes_[1] + zz * ybdes_[2];
      R[2][2] = zx * zbdes_[0] + zy * zbdes_[1] + zz * zbdes_[2];

      // (Rd^T x R - R^T x Rd) * 0.5 ([0,a,b; - a,0,c; - b, - c,0]).
      double eR[3];
      eR[0] = -(R[2][1] - R[1][2]) * 0.5;
      eR[1] =  (R[2][0] - R[0][2]) * 0.5;
      eR[2] = -(R[1][0] - R[0][1]) * 0.5;

      // Apply control.
      roll_torque_ = -kKR * eR[0] - kKW * (rolldot_);
      pitch_torque_ = -kKR * eR[1] - kKW * (pitchdot_);
      yaw_torque_ = -kKRYaw * eR[2] - kKWYaw * (yawdot_);
    }
  }
  ComputeDesiredRPM();
}

void Quadrotor::SetControlInputs(double up_force, double roll_torque, double pitch_torque, double yaw_torque) {
  up_force_ = up_force;
  roll_torque_ = roll_torque;
  pitch_torque_ = pitch_torque;
  yaw_torque_ = yaw_torque;
}

void Quadrotor::SetGoal(double x, double y, double z) {
  goal_x_ = x;
  goal_y_ = y;
  goal_z_ = z;
}

void Quadrotor::Draw(VisualizerWindow* window) {
  // Reload compiled lists if needed.
  if (!lists_created_ || window->HasContextChanged()) {
    if (lists_created_ && robot_dl_ != 0) {
      glDeleteLists(robot_dl_, 1);
    }
    if (lists_created_ && rotor_dl_ != 0) {
      glDeleteLists(rotor_dl_, 1);
    }
    robot_model_.reset(new Model3DS());
    robot_model_->Load(kQuadModelPath);
    robot_dl_ = glGenLists(1);
    glNewList(robot_dl_, GL_COMPILE);
    robot_model_->Draw();
    glEndList();
    rotor_model_.reset(new Model3DS());
    rotor_model_->Load(kRotorModelPath);
    rotor_dl_ = glGenLists(1);
    glNewList(rotor_dl_, GL_COMPILE);
    rotor_model_->Draw();
    glEndList();
    lists_created_ = true;
  }

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  // Z - X - Y convention -> roll, pitch, yaw
  float c1 = cos(roll_);
  float c2 = cos(pitch_);
  float c3 = cos(yaw_);
  float s1 = sin(roll_);
  float s2 = sin(pitch_);
  float s3 = sin(yaw_);
  // Compute rotation matrix
  GLfloat m[16];
  m[3] = 0.0;
  m[7] = 0.0;
  m[11] = 0.0;
  m[12] = 0.0;
  m[13] = 0.0;
  m[14] = 0.0;
  m[15] = 1.0;
  m[0] = c2 * c3;
  m[2] = -c2 * s3;
  m[1] = -s2;
  m[4] = c1 * c3 * s2 + s1 * s3;
  m[6] = c3 * s1 - c1 * s2 * s3;
  m[5] = c1 * c2;
  m[8] = -(c3 * s1 * s2 - c1 * s3);
  m[10] = -(-c1 * c3 - s1 * s2 * s3);
  m[9] = -(c2 * s1);
  glTranslatef(x_, z_, -y_);

  // Draw axis
  if (FLAGS_quadrotor_axes) {
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(m[0] * 0.5, m[1] * 0.5, m[2] * 0.5);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(-m[8] * 0.5, -m[9] * 0.5, -m[10] * 0.5);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(m[4] * 0.5, m[5] * 0.5, m[6] * 0.5);
    glEnd();
    glBegin(GL_LINES);
    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(xbdes_[0] * 0.5, xbdes_[2] * 0.5, -xbdes_[1] * 0.5);
    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(ybdes_[0] * 0.5, ybdes_[2] * 0.5, -ybdes_[1] * 0.5);
    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(zbdes_[0] * 0.5, zbdes_[2] * 0.5, -zbdes_[1] * 0.5);
    glEnd();
    glEnable(GL_LIGHTING);
  }

  // Draw the quadrotor.
  glMultMatrixf(m);
  glColor3f(0.5 * std::get<0>(color_), 0.5 * std::get<1>(color_), 0.5 * std::get<2>(color_));
  double xv, yv, zv;
  window->GetCenterOfAttention(&xv, &yv, &zv);
  float dist = sqrtf((x_ - xv) * (x_ - xv) + (y_ - zv) * (y_ - zv));
  if (dist < 10.0 && !window->IsFastModeEnabled()) {
    glCallList(robot_dl_);
    glPushMatrix();
    glTranslatef(0.2552, 0.0, 0.0);
    glRotatef(-r1a_ / M_PI * 180.0, 0.0, 1.0, 0.0); // Those rotate CW
    glCallList(rotor_dl_);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.257, 0.0, 0.0);
    glRotatef(-r3a_ / M_PI * 180.0, 0.0, 1.0, 0.0); // Those rotate CW
    glCallList(rotor_dl_);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.237);
    glRotatef(r4a_ / M_PI * 180.0, 0.0, 1.0, 0.0);
    glCallList(rotor_dl_);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.0, 0.0, -0.255);
    glRotatef(r2a_ / M_PI * 180.0, 0.0, 1.0, 0.0);
    glCallList(rotor_dl_);
    glPopMatrix();
  } else {
    glPushMatrix();
    glScalef(kLengthArm, kHeightArm, kWidthArm);
    glutWireCube(1.0);
    glPopMatrix();
    glPushMatrix();
    glScalef(kWidthArm, kHeightArm, kLengthArm);
    glutWireCube(1.0);
    glPopMatrix();
  }

  glPopMatrix();
}

void Quadrotor::ComputeDesiredRPM() {
  // Solving the linear equation.
  w1_desired_ = (kKM * kHalfArmLength * up_force_ - 2.0 * kKM * pitch_torque_ + kKF * kHalfArmLength * yaw_torque_) / (4.0 * kKF * kKM * kHalfArmLength);
  w1_desired_ = std::max(0.0, w1_desired_);
  w1_desired_ = sqrt(w1_desired_);
  w1_desired_ = std::min(kMaxRotorSpeed, std::max(kMinRotorSpeed, w1_desired_));

  w2_desired_ = 0.25 * ((up_force_ + 2.0 * roll_torque_ / kHalfArmLength) / kKF - yaw_torque_ / kKM);
  w2_desired_ = std::max(0.0, w2_desired_);
  w2_desired_ = sqrt(w2_desired_);
  w2_desired_ = std::min(kMaxRotorSpeed, std::max(kMinRotorSpeed, w2_desired_));

  w3_desired_ = (kKM * kHalfArmLength * up_force_ + 2.0 * kKM * pitch_torque_ + kKF * kHalfArmLength * yaw_torque_) / (4.0 * kKF * kKM * kHalfArmLength);
  w3_desired_ = std::max(0.0, w3_desired_);
  w3_desired_ = sqrt(w3_desired_);
  w3_desired_ = std::min(kMaxRotorSpeed, std::max(kMinRotorSpeed, w3_desired_));

  w4_desired_ = 0.25 * ((up_force_ - 2.0 * roll_torque_ / kHalfArmLength) / kKF - yaw_torque_ / kKM);
  w4_desired_ = std::max(0.0, w4_desired_);
  w4_desired_ = sqrt(w4_desired_);
  w4_desired_ = std::min(kMaxRotorSpeed, std::max(kMinRotorSpeed, w4_desired_));
}
