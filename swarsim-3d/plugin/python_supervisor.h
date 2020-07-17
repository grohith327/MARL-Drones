#ifndef _PYTHON_SUPERVISOR_H
#define _PYTHON_SUPERVISOR_H

#include <Python.h>

#include "core/supervisor.h"
#include "util/registerer.h"

class PythonSupervisor : public Supervisor {
  REGISTER("PythonSupervisor", Supervisor);

 private:
  bool Initialize() override;
  void Destroy() override;
  void Update(double t, double dt) override;
  void Draw(double t, VisualizerWindow* window) override;

  PyObject* instance_;
};

#endif
