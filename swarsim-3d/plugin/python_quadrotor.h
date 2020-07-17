#ifndef _PYTHON_QUADROTOR_H
#define _PYTHON_QUADROTOR_H

#include <Python.h>

#include "core/quadrotor.h"
#include "core/robot.h"
#include "display/window.h"
#include "util/registerer.h"

class PythonQuadrotor : public Quadrotor {
  REGISTER("PythonQuadrotor", Robot);

 private:
  bool Initialize() override;
  void Execute(double t, double dt) override;
  bool GoalBased() override;
};

#endif
