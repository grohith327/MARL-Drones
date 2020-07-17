#include "python_supervisor.h"
#include "python_unicycle.h"

#include <iostream>
#include <sstream>
#include <string>

#include "gflags/gflags.h"
#include "util/utils.h"

DECLARE_int32(nthreads);
DEFINE_string(python_supervisor_arg, "", "String argument passed to the start() method.");
DEFINE_string(python_supervisor_script, "", "Python script filename.");


// Helper functions to display information using OpenGL.
// Some functions are already available from the PyOpenGL package.
static PyObject* swarmui_glColor3f(PyObject* self, PyObject* args) {
  double r, g, b;
  if (!PyArg_ParseTuple(args, "ddd", &r, &g, &b))
    return nullptr;
  glColor3f(r, g, b);
  Py_RETURN_NONE;
}

static PyObject* swarmui_glBegin(PyObject* self, PyObject* args) {
  int v;
  if (!PyArg_ParseTuple(args, "i", &v))
    return nullptr;
  glBegin(v);
  Py_RETURN_NONE;
}

static PyObject* swarmui_glEnd(PyObject* self, PyObject* args) {
  glEnd();
  Py_RETURN_NONE;
}

static PyObject* swarmui_glLineWidth(PyObject* self, PyObject* args) {
  double v;
  if (!PyArg_ParseTuple(args, "d", &v))
    return nullptr;
  glLineWidth(v);
  Py_RETURN_NONE;
}

static PyMethodDef kSwarmuiMethods[] = {
    {"glColor3f", swarmui_glColor3f, METH_VARARGS, "glColor3f"},
    {"glLineWidth", swarmui_glLineWidth, METH_VARARGS, "glLineWidth"},
    {"glBegin", swarmui_glBegin, METH_VARARGS, "glBegin"},
    {"glEnd", swarmui_glEnd, METH_VARARGS, "glEnd"},
    {NULL, NULL, 0, NULL}
};

namespace {
static std::string GetDirectory(const std::string& filename) {
  size_t found;
  found = filename.find_last_of("/\\");
  return filename.substr(0, found);
}
}

bool PythonSupervisor::Initialize() {
  FLAGS_nthreads = 1;  // Because of the GIL.
  Py_Initialize();
  Py_InitModule("swarmui", kSwarmuiMethods);

  if (FLAGS_python_supervisor_script.empty()) {
    std::cerr << "Python script driving the robots must be set with --python_supervisor_script" << std::endl;
    return false;
  }
  // TODO: Start Python script.
  std::ostringstream buffer;
  buffer << "import imp\n"
         << "imp.load_source('swarmsim', r'./plugin/swarmsim.py')\n"
         << "import sys\n"
         << "sys.path.insert(0, '" << GetDirectory(FLAGS_python_supervisor_script) << "')\n"
         << "imp.load_source('robot', r'" << FLAGS_python_supervisor_script << "')\n";
  PyRun_SimpleString(buffer.str().c_str());

  PyObject* module = PyImport_ImportModule("robot");
  if (!module) {
    std::cerr << "Unable to load module at \"" << FLAGS_python_supervisor_script << "\"" << std::endl;
    return false;
  }

  // Get supervisor.
  PyObject* supervisor_class = PyObject_GetAttrString(module, "Supervisor");
  if (!supervisor_class) {
    std::cerr << "Unable to find \"Supervior\" class" << std::endl;
    return false;
  }
  instance_ = PyObject_CallObject(supervisor_class, nullptr);
  if (!instance_) {
    std::cerr << "Unable to call \"Supervior.__init__()\" class" << std::endl;
    PyErr_Print();
    return false;
  }
  PyObject* robot_list = PyObject_CallMethod(instance_, "initialize", "(s)", FLAGS_python_supervisor_arg.c_str());
  if (!robot_list) {
    PyErr_Print();
    return false;
  }

  // Go through the robot list.
  const int n = PyList_Size(robot_list);
  for (int i = 0; i < n; ++i) {
    PyObject* instance = PyList_GetItem(robot_list, i);
    if (!instance) {
      PyErr_Print();
      return false;
    }
    PyObject* result = PyObject_CallMethod(instance, "type", nullptr);
    if (!result) {
      PyErr_Print();
      return false;
    }
    Robot* robot = CreateRobot(PyString_AsString(result));
    Py_DECREF(result);

    PyObject* x = PyObject_GetAttrString(instance, "x");
    PyObject* y = PyObject_GetAttrString(instance, "y");
    PyObject* z = PyObject_GetAttrString(instance, "z");
    PyObject* yaw = PyObject_GetAttrString(instance, "yaw");
    PyObject* pitch = PyObject_GetAttrString(instance, "pitch");
    PyObject* roll = PyObject_GetAttrString(instance, "roll");
    PyObject* type = PyObject_GetAttrString(instance, "identifier");
    if (!(x && y && z && yaw && pitch && roll && type)) {
      PyErr_Print();
      return false;
    }
    if (PyObject_SetAttrString(instance, "flying", PyBool_FromLong(robot->IsFlying())) != 0) {
      PyErr_Print();
      return false;
    }

    robot->SetPosition(PyFloat_AsDouble(x),
                       PyFloat_AsDouble(y),
                       PyFloat_AsDouble(z),
                       PyFloat_AsDouble(roll),
                       PyFloat_AsDouble(pitch),
                       PyFloat_AsDouble(yaw));
    robot->SetType(PyLong_AsLong(type));
    robot->SetPythonInstance(instance);
    Py_DECREF(x);
    Py_DECREF(y);
    Py_DECREF(z);
    Py_DECREF(yaw);
    Py_DECREF(pitch);
    Py_DECREF(roll);
    Py_DECREF(type);

    // Call initialize method.
    result = PyObject_CallMethod(instance, "initialize", nullptr);
    if (!result) {
      std::cerr << "Unable to call \"Robot.initialize()\"" << std::endl;
      PyErr_Print();
      return false;
    }
    Py_DECREF(result);
  }
  return true;
}

void PythonSupervisor::Update(double t, double dt) {
  PyObject* result = PyObject_CallMethod(instance_, "execute", "(ff)", t, dt);
  if (!result) {
    std::cerr << "Unable to call \"Supervisor.execute()\"" << std::endl;
    PyErr_Print();
    return;
  }
  Py_DECREF(result);

  // The supervisor may have changed the positions of robots.
  for (int i = 0; i < NumRobots(); ++i) {
    Robot* robot = GetMutableRobot(i);
    PyObject* instance = robot->python_instance();
    PyObject* x = PyObject_GetAttrString(instance, "x");
    PyObject* y = PyObject_GetAttrString(instance, "y");
    PyObject* z = PyObject_GetAttrString(instance, "z");
    PyObject* yaw = PyObject_GetAttrString(instance, "yaw");
    PyObject* pitch = PyObject_GetAttrString(instance, "pitch");
    PyObject* roll = PyObject_GetAttrString(instance, "roll");
    if (!(x && y && z && yaw && pitch && roll)) {
      PyErr_Print();
      return;
    }
    robot->SetPosition(PyFloat_AsDouble(x),
                       PyFloat_AsDouble(y),
                       PyFloat_AsDouble(z),
                       PyFloat_AsDouble(roll),
                       PyFloat_AsDouble(pitch),
                       PyFloat_AsDouble(yaw));
    Py_DECREF(x);
    Py_DECREF(y);
    Py_DECREF(z);
    Py_DECREF(yaw);
    Py_DECREF(pitch);
    Py_DECREF(roll);
  }
}

void PythonSupervisor::Destroy() {
  for (int i = 0; i < NumRobots(); ++i) {
    GetMutableRobot(i)->ClosePython();
  }
  Py_Finalize();
}

void PythonSupervisor::Draw(double t, VisualizerWindow* window) {
  PyObject* result = PyObject_CallMethod(instance_, "draw", "(f)", t);
  if (!result) {
    std::cerr << "Unable to call \"Supervisor.draw()\"" << std::endl;
    PyErr_Print();
    return;
  }
  Py_DECREF(result);
}
