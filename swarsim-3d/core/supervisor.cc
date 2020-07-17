#include "supervisor.h"

#include <algorithm>
#include <iostream>
#include <thread>

#include "gflags/gflags.h"
#include "util/registerer.h"
#include "util/utils.h"

DEFINE_int32(nearest_neighbors, 4, "Number of nearest neighbors passed to each robot.");
DEFINE_int32(nthreads, 2, "Number of threads used to update robots.");

DECLARE_string(supervisor);

bool Supervisor::Init() {
  if (!Initialize()) {
    return false;
  }
  // Check that at least one robot was created.
  if (robots_.empty()) {
    std::cerr << "No robots were created by \"" << FLAGS_supervisor << "\"." << std::endl;
  }

  // for (auto it = robots_.begin(); it != robots_.end(); ++it)
  // {
  //   Robot* robot = it->get();
  //   std::cout<<"Robot type:"<<robot->type()<<std::endl;
  // }

  // Nearest neighbors.
  robot_points_ = annAllocPts(robots_.size(), 2);

  // Assign colors to the different robots.
  RandomColors colors(0.5);
  robot_type_colors_.clear();
  for (auto it = robots_.begin(); it != robots_.end(); ++it) {
    Robot* robot = it->get();
    auto it_colors = robot_type_colors_.find(robot->type());
    // Color already picked.
    if (it_colors != robot_type_colors_.end()) {
      robot->SetColor(it_colors->second);
      continue;
    }
    const auto color = colors.Next();
    robot->SetColor(color);
    robot_type_colors_[robot->type()] = color;
  }

  return true;
}

void Supervisor::Close() {
  Destroy();
  annClose();
}

void Supervisor::DrawState(double t, VisualizerWindow* window) {
  // Draw robots.
  for (auto it = robots_.begin(); it != robots_.end(); ++it) {
    Robot* robot = it->get();
    robot->Draw(window);
  }
  // Draw extra info from derived classes.
  Draw(t, window);
}

// static
void Supervisor::ThreadStep(const Supervisor* self, int id, double t, double dt) {
  // Perform steps for all robots.
  for (int i = id; i < self->robots_.size(); i += FLAGS_nthreads) {
    self->robots_[i]->Step(t, dt);
  }
}

void Supervisor::Step(double t, double dt) {
  // Variables needed for ANN.
  ANNidx neighbor_indices[FLAGS_nearest_neighbors + 1];
  ANNdist neighbor_distances[FLAGS_nearest_neighbors + 1];
  int nneighbors = std::min<int>(FLAGS_nearest_neighbors, robots_.size() - 1);
  std::unique_ptr<ANNkd_tree> kd_tree(new ANNkd_tree(robot_points_, robots_.size(), 2));
  // Find closest neighbors.
  for (int i = 0; i < robots_.size(); ++i) {
    robots_[i]->ClearNeighbors();
    kd_tree->annkSearch(robot_points_[i], nneighbors + 1, neighbor_indices, neighbor_distances, 0.0);
    for (int j = 0; j < nneighbors + 1; ++j) {
      if (neighbor_indices[j] == i || neighbor_indices[j] < 0) continue;
      robots_[i]->AddNeighbor(robots_[neighbor_indices[j]].get());
    }
  }

  // Start threading.
  std::vector<std::thread> workers;
  for (int i = 0; i < FLAGS_nthreads; ++i) {
    workers.push_back(std::thread(ThreadStep, this, i, t, dt));
  }

  // Finish threading.
  for (std::thread &t : workers) {
    if (t.joinable()) {
      t.join();
    }
  }

  // Update robot positions for nearest neighbor search.
  for (int i = 0; i < robots_.size(); i++) {
    robot_points_[i][0] = robots_[i]->x();
    robot_points_[i][1] = robots_[i]->y();
  }

  // Notify derived class.
  Update(t, dt);
}

Robot* Supervisor::CreateRobot(const std::string& robot_type) {
  if (!Registry<Robot>::CanNew(robot_type)) {
    std::cerr << "Robot type \"" << robot_type << "\" not supported. List of supported types:" <<  std::endl;
    for (const std::string& k : Registry<Robot>::GetKeys()) {
      std::cerr << "  " << k << std::endl;
    }
    return nullptr;
  }
  std::unique_ptr<Robot> robot = Registry<Robot>::New(robot_type);
  if (!robot->Init()) {
    return nullptr;
  }
  robots_.emplace_back(std::move(robot));
  return robots_.back().get();
}

int Supervisor::NumRobots() {
  return robots_.size();
}

const Robot& Supervisor::GetRobot(int i) {
  return *robots_[i];
}

Robot* Supervisor::GetMutableRobot(int i) {
  return robots_[i].get();
}
