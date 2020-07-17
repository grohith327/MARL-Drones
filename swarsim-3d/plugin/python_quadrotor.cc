#include "python_quadrotor.h"

#include <iostream>

bool PythonQuadrotor::Initialize() {
  return true;
}

void PythonQuadrotor::Execute(double t, double dt) {
  // Update attributes.
  PyObject* xdot = PyFloat_FromDouble(this->xdot_);
  PyObject* ydot = PyFloat_FromDouble(this->ydot_);
  PyObject* zdot = PyFloat_FromDouble(this->zdot_);
  PyObject* yawdot = PyFloat_FromDouble(this->yawdot_);
  PyObject* pitchdot = PyFloat_FromDouble(this->pitchdot_);
  PyObject* rolldot = PyFloat_FromDouble(this->rolldot_);
  if (PyObject_SetAttrString(python_instance_, "xdot", xdot) != 0 ||
      PyObject_SetAttrString(python_instance_, "ydot", ydot) != 0 ||
      PyObject_SetAttrString(python_instance_, "zdot", zdot) != 0 ||
      PyObject_SetAttrString(python_instance_, "yawdot", yawdot) != 0 ||
      PyObject_SetAttrString(python_instance_, "pitchdot", pitchdot) != 0 ||
      PyObject_SetAttrString(python_instance_, "rolldot", rolldot) != 0) {
    PyErr_Print();
    return;
  }
  Py_DECREF(xdot);
  Py_DECREF(ydot);
  Py_DECREF(zdot);
  Py_DECREF(yawdot);
  Py_DECREF(pitchdot);
  Py_DECREF(rolldot);


  PyObject* result = ExecutePython(t, dt);
  if (!result) {
    return;
  }
  double up_force, roll_torque, pitch_torque, yaw_torque;
  PyArg_ParseTuple(result, "dddd", &up_force, &roll_torque, &pitch_torque, &yaw_torque);
  Py_DECREF(result);
  SetControlInputs(up_force, roll_torque, pitch_torque, yaw_torque);
}

bool PythonQuadrotor::GoalBased() {
  return false;
}
