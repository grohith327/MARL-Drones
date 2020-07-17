#ifndef _SUPERVISOR_H
#define _SUPERVISOR_H

#include <memory>
#include <unordered_map>
#include <vector>

#include "ann/ANN.h"
#include "core/robot.h"
#include "display/window.h"

class Supervisor {
 public:
  // Initializes the robots.
  bool Init();

  // Closes the supervisor.
  void Close();

  // Executes a step.
  void Step(double t, double dt);

  // Draws robots on the screen.
  void DrawState(double t, VisualizerWindow* window);

  // Robots.
  int NumRobots();
  const Robot& GetRobot(int i);

 protected:
  // Adds a new robot of a given type. The supervisor keeps ownership of the robot.
  Robot* CreateRobot(const std::string& robot_type);

  // Get mutable robot.
  Robot* GetMutableRobot(int i);

  // Get robot colors.
  const std::unordered_map<int, std::tuple<float, float, float>>& RobotColors() const { return robot_type_colors_; };

 private:
  // Private function to initialize the robots.
  virtual bool Initialize() = 0;

  // Private function to destroy the robots.
  virtual void Destroy() {}

  // Draws some extra info on the screen.
  // The window can be used to move the camera around.
  virtual void Draw(double t, VisualizerWindow* window) {}

  // Informs the supervisor that a step was taken.
  // t is the current simulation time.
  virtual void Update(double t, double dt) {}

  // Thread function.
  static void ThreadStep(const Supervisor* self, int id, double t, double dt);

  // Robots.
  std::vector<std::unique_ptr<Robot>> robots_;

  // Used for nearest neighbor.
  ANNpointArray robot_points_;

  // Colors of robots in case derived classes needed to use it for drawing info.
  std::unordered_map<int, std::tuple<float, float, float>> robot_type_colors_;
};

#endif
