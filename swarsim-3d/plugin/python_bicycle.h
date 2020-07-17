#ifndef _PYTHON_BICYCLE_H
#define _PYTHON_BICYCLE_H

#include <Python.h>

#include "core/bicycle.h"
#include "core/robot.h"
#include "display/window.h"
#include "util/registerer.h"

class PythonBicycle : public Bicycle {
  REGISTER("PythonBicycle", Robot);

 private:
  bool Initialize() override;
  void Execute(double t, double dt) override;
  bool GoalBased() override;
};

#endif
