#include "python_unicycle.h"

#include <iostream>

bool PythonUnicycle::Initialize() {
  return true;
}

void PythonUnicycle::Execute(double t, double dt) {
  PyObject* result = ExecutePython(t, dt);
  if (!result) {
    return;
  }
  double u, w;
  PyArg_ParseTuple(result, "dd", &u, &w);
  Py_DECREF(result);
  SetControlInputs(u, w);
}

bool PythonUnicycle::GoalBased() {
  return false;
}
