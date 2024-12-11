// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_msgs:msg/Environment.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_msgs/msg/detail/environment__struct.h"
#include "robot_msgs/msg/detail/environment__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_msgs__msg__environment__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robot_msgs.msg._environment.Environment", full_classname_dest, 39) == 0);
  }
  robot_msgs__msg__Environment * ros_message = _ros_message;
  {  // temp
    PyObject * field = PyObject_GetAttrString(_pymsg, "temp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pressure
    PyObject * field = PyObject_GetAttrString(_pymsg, "pressure");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pressure = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // humidity
    PyObject * field = PyObject_GetAttrString(_pymsg, "humidity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->humidity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->altitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_msgs__msg__environment__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Environment */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_msgs.msg._environment");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Environment");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_msgs__msg__Environment * ros_message = (robot_msgs__msg__Environment *)raw_ros_message;
  {  // temp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pressure
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pressure);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pressure", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // humidity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->humidity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "humidity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
