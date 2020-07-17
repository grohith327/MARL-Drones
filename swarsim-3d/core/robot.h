#ifndef _ROBOT_H
#define _ROBOT_H

#include <Python.h>

#include <tuple>
#include <vector>

#include "display/window.h"

class Robot {
 public:
  Robot();

  // Initializes the robots.
  virtual bool Init();

  // Executes a step.
  virtual void Step(double t, double dt);

  // Draw the robot.
  virtual void Draw(VisualizerWindow* window) = 0;

  // Tells whether flying or ground robot.
  virtual bool IsFlying() const = 0;
  virtual bool IsGround() const = 0;

  // Position of the robot.
  void SetPosition(double x, double y, double yaw);
  void SetPosition(double x, double y, double z, double roll, double pitch, double yaw);
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double yaw() const { return yaw_; }
  double roll() const { return roll_; }
  double pitch() const { return pitch_; }

  // Robot type.
  void SetType(int i);
  int type() const { return type_; }

  // Closest neighbors.
  void ClearNeighbors();
  void AddNeighbor(const Robot* neighbor);
  const std::vector<const Robot*>& neighbors() const { return neighbors_; }

  // Notifies derived classes to display some color on the robot.
  void SetColor(const std::tuple<float, float, float>& color);
  const std::tuple<float, float, float>& color() const { return color_; }

  // Sets the underlying Python instance.
  void SetPythonInstance(PyObject* instance);
  PyObject* python_instance() const;
  PyObject* ExecutePython(double t, double dt);
  void ClosePython();

 private:
  friend class Unicycle;
  friend class Bicycle;
  friend class Quadrotor;
  friend class PythonQuadrotor;

  // Private function to initialize the robots.
  virtual bool Initialize(int robot_type) = 0;
  virtual void Execute(double t, double dt, int robot_type) = 0;

  // Determines whether a derived class uses the goal-based control or
  // controls the velocities directly. Default is to provide control
  // inputs. Direct derived classes must provide these two types of control.
  virtual bool GoalBased() { return false; }

  // Positions.
  double x_;  // Easting.
  double y_;  // Northing.
  double z_;  // Altitude.
  double yaw_;  // Around z (up).
  double roll_;  // Around local x (front).
  double pitch_;  // Around local y (left).

  // Type.
  int type_;

  // Neighbors.
  std::vector<const Robot*> neighbors_;

  // Drawing.
  std::tuple<float, float, float> color_;

  // Python bindings.
  PyObject* python_instance_;
};

#endif
