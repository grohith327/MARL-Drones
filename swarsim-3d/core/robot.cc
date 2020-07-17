#include "robot.h"

#include <iostream>

Robot::Robot()
    : x_(0.0), y_(0.0), z_(0.0), yaw_(0.0), roll_(0.0), pitch_(0.0), type_(0), python_instance_(nullptr) {}

bool Robot::Init() {
  return Initialize(type());
}

void Robot::Step(double t, double dt) {
  Execute(t, dt, type());
}

void Robot::SetPosition(double x, double y, double yaw) {
  x_ = x;
  y_ = y;
  yaw_ = yaw;
}

void Robot::SetPosition(double x, double y, double z, double roll, double pitch, double yaw) {
  x_ = x;
  y_ = y;
  z_ = z;
  yaw_ = yaw;
  pitch_ = pitch;
  roll_ = roll;
}

void Robot::ClearNeighbors() {
  neighbors_.clear();
}

void Robot::AddNeighbor(const Robot* neighbor) {
  neighbors_.push_back(neighbor);
}

void Robot::SetType(int type) {
  type_ = type;
}

void Robot::SetColor(const std::tuple<float, float, float>& color) {
  color_ = color;
}

void Robot::SetPythonInstance(PyObject* instance) {
  python_instance_ = instance;
}

PyObject* Robot::python_instance() const {
  return python_instance_;
}

PyObject* Robot::ExecutePython(double t, double dt) {
  if (!python_instance_) {
    std::cerr << "SetPythonInstance() not called" << std::endl;
    return nullptr;
  }

  // Update attributes.
  PyObject* x = PyFloat_FromDouble(this->x());
  PyObject* y = PyFloat_FromDouble(this->y());
  PyObject* z = PyFloat_FromDouble(this->z());
  PyObject* yaw = PyFloat_FromDouble(this->yaw());
  PyObject* pitch = PyFloat_FromDouble(this->pitch());
  PyObject* roll = PyFloat_FromDouble(this->roll());
  if (PyObject_SetAttrString(python_instance_, "x", x) != 0 ||
      PyObject_SetAttrString(python_instance_, "y", y) != 0 ||
      PyObject_SetAttrString(python_instance_, "z", z) != 0 ||
      PyObject_SetAttrString(python_instance_, "yaw", yaw) != 0 ||
      PyObject_SetAttrString(python_instance_, "pitch", pitch) != 0 ||
      PyObject_SetAttrString(python_instance_, "roll", roll) != 0) {
    PyErr_Print();
    return nullptr;
  }
  Py_DECREF(x);
  Py_DECREF(y);
  Py_DECREF(z);
  Py_DECREF(yaw);
  Py_DECREF(pitch);
  Py_DECREF(roll);


  // Update neighbors.
  PyObject* list = PyList_New(neighbors().size());
  for (int i = 0; i < neighbors().size(); ++i) {
    if (PyList_SetItem(list, i, neighbors()[i]->python_instance()) != 0) {
      PyErr_Print();
      return nullptr;
    }
  }
  if (PyObject_SetAttrString(python_instance_, "neighbors", list) != 0) {
    PyErr_Print();
    return nullptr;
  }
  // Py_DECREF(list);

  PyObject* result = PyObject_CallMethod(python_instance_, "execute", "(ff)", t, dt);
  if (!result) {
    std::cerr << "Unable to call \"Robot.execute()\"" << std::endl;
    PyErr_Print();
    return nullptr;
  }
  return result;
}

void Robot::ClosePython() {
  if (!python_instance_) {
    std::cerr << "SetPythonInstance() not called" << std::endl;
    return;
  }
  PyObject* result = PyObject_CallMethod(python_instance_, "close", nullptr);
  if (!result) {
    std::cerr << "Unable to call \"Robot.close()\"" << std::endl;
    PyErr_Print();
    return;
  }
  return;
}
