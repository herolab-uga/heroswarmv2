// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_msgs:msg/DistanceSensor.idl
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
#include "robot_msgs/msg/detail/distance_sensor__struct.h"
#include "robot_msgs/msg/detail/distance_sensor__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_msgs__msg__distance_sensor__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("robot_msgs.msg._distance_sensor.DistanceSensor", full_classname_dest, 46) == 0);
  }
  robot_msgs__msg__DistanceSensor * ros_message = _ros_message;
  {  // sensor1
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor1 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // sensor2
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor2 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // sensor3
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor3 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // sensor4
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor4");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor4 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_msgs__msg__distance_sensor__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DistanceSensor */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_msgs.msg._distance_sensor");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DistanceSensor");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_msgs__msg__DistanceSensor * ros_message = (robot_msgs__msg__DistanceSensor *)raw_ros_message;
  {  // sensor1
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sensor1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensor2
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sensor2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensor3
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sensor3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensor4
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sensor4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
