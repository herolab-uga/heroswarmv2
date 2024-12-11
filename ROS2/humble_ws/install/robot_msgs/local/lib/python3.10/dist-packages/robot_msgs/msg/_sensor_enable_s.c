// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_msgs:msg/SensorEnable.idl
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
#include "robot_msgs/msg/detail/sensor_enable__struct.h"
#include "robot_msgs/msg/detail/sensor_enable__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_msgs__msg__sensor_enable__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("robot_msgs.msg._sensor_enable.SensorEnable", full_classname_dest, 42) == 0);
  }
  robot_msgs__msg__SensorEnable * ros_message = _ros_message;
  {  // environment
    PyObject * field = PyObject_GetAttrString(_pymsg, "environment");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->environment = (Py_True == field);
    Py_DECREF(field);
  }
  {  // imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->imu = (Py_True == field);
    Py_DECREF(field);
  }
  {  // light
    PyObject * field = PyObject_GetAttrString(_pymsg, "light");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->light = (Py_True == field);
    Py_DECREF(field);
  }
  {  // proximity
    PyObject * field = PyObject_GetAttrString(_pymsg, "proximity");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->proximity = (Py_True == field);
    Py_DECREF(field);
  }
  {  // mic
    PyObject * field = PyObject_GetAttrString(_pymsg, "mic");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->mic = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_msgs__msg__sensor_enable__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SensorEnable */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_msgs.msg._sensor_enable");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SensorEnable");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_msgs__msg__SensorEnable * ros_message = (robot_msgs__msg__SensorEnable *)raw_ros_message;
  {  // environment
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->environment ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "environment", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->imu ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->light ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // proximity
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->proximity ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "proximity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mic
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->mic ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
