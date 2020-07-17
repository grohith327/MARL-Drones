#include "default_quadrotor.h"

#include <cmath>
#include <iostream>

#include "util/utils.h"

namespace {
constexpr double kDistanceThreshold = 0.15;
constexpr double kGoalRadius = 0.05;
}  // namespace

DefaultQuadrotor::DefaultQuadrotor()
{
  count = 0;
  // int pos = 0;
  // for(double i = -2.0; i <= 2.0; i+=0.4)
  // {
  //   for(double j = 2.0; j>=-2.0; j -=0.4)
  //   {
  //     positions[pos][0] = i;
  //     positions[pos][1] = j;
  //     pos++;
  //   }
  // }

  // std::cout<<"Robot Type: "<<Robot::type()<<std::endl;

  // for(int i=0;i<121;i++)
  // {
  //   std::cout<<positions[i][0]<<" "<<positions[i][1]<<std::endl;
  // }
}

bool DefaultQuadrotor::Initialize(int robot_type) {
  // Pick a random goal point .
  // goal_x_1 = RandomUniform() * 4.0 - 1.5;
  // goal_y_1 = RandomUniform() * 4.0 - 1.5;
  // goal_z_1 = RandomUniform() * 1.0 + 1.0;
  // std::cout<<"Robot Type:"<<robot_type<<std::endl;

  goal_x_ = RandomUniform() * 4.0 - 1.5;
  goal_y_ = RandomUniform() * 4.0 - 1.5;
  goal_z_ = 1.5;
  count++;
  return true;
}

void DefaultQuadrotor::Execute(double t, double dt, int robot_type) {

  // std::cout<<"X:"<<goal_x_<<" Y:"<<goal_y_<<" Z:"<<goal_z_<<std::endl;

  // std::cout<<"Robot Type:"<<robot_type<<std::endl;


  if(robot_type == 0)
  {
    for(double i = -2.0; i <= -0.4; i+=0.4)
    {
      for(double j = 2.0; j>=-2.0; j -=0.4)
      {
        std::vector<int> temp;
        temp.push_back(i);
        temp.push_back(j);
        positions.push_back(temp);
      }
    }
  }
  else
  {
    for(double i = 0.0; i <= 2.0; i+=0.4)
    {
      for(double j = 2.0; j>=-2.0; j -=0.4)
      {
        std::vector<int> temp;
        temp.push_back(i);
        temp.push_back(j);
        positions.push_back(temp);
      }
    }
  }
  
  double dx = goal_x_ - x();
  double dy = goal_y_ - y();
  double dz = goal_z_ - z();
  double e = sqrtf(dx * dx + dy * dy + dz * dz);

  while (e < kDistanceThreshold) {

    // std::cout<<"Execute "<<count<<std::endl;
    // count++;
    // goal_x_1 = RandomUniform() * 4.0 - 1.5;
    // goal_y_1 = RandomUniform() * 4.0 - 1.5;
    // goal_z_1 = RandomUniform() * 1.0 + 1.0;
    goal_x_ = positions[count][0];
    goal_y_ = positions[count][1];
    goal_z_ = 1.5;
    count++;
    if(robot_type == 0 && count == 55)
    {
      count = 0;
    }
    if(robot_type == 1 && count == 66)
    {
      count = 0;
    }
    dx = goal_x_ - x();
    dy = goal_y_ - y();
    dz = goal_z_ - z();
    e = sqrtf(dx * dx + dy * dy + dz * dz);
  }
  // Go to goal point.
  SetGoal(goal_x_, goal_y_, goal_z_);
}

void DefaultQuadrotor::Draw(VisualizerWindow* window) {
  Quadrotor::Draw(window);

  glPushMatrix();
  glTranslatef(goal_x_, goal_z_, -goal_y_);
  // glTranslatef(goal_x_2, goal_z_2, -goal_y_2);
  auto robot_color = color();
  glColor4f(std::get<0>(robot_color) * 0.5, std::get<1>(robot_color) * 0.5, std::get<2>(robot_color) * 0.5, 0.6f);
  glutWireSphere(kGoalRadius, 8, 8);
  glPopMatrix();
}
