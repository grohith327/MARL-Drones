#ifndef _PYTHON_UNICYCLE_H
#define _PYTHON_UNICYCLE_H

#include <Python.h>

#include "core/unicycle.h"
#include "core/robot.h"
#include "display/window.h"
#include "util/registerer.h"

class PythonUnicycle : public Unicycle {
  REGISTER("PythonUnicycle", Robot);

 private:
  bool Initialize() override;
  void Execute(double t, double dt) override;
  bool GoalBased() override;
};

#endif
