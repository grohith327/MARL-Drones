#include "python_bicycle.h"

#include <iostream>

bool PythonBicycle::Initialize() {
  return true;
}

void PythonBicycle::Execute(double t, double dt) {
  PyObject* result = ExecutePython(t, dt);
  if (!result) {
    return;
  }
  double speed, steering;
  PyArg_ParseTuple(result, "dd", &speed, &steering);
  Py_DECREF(result);
  SetControlInputs(speed, steering);
}

bool PythonBicycle::GoalBased() {
  return false;
}
