// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_msgs:msg/DockEnable.idl
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
#include "robot_msgs/msg/detail/dock_enable__struct.h"
#include "robot_msgs/msg/detail/dock_enable__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_msgs__msg__dock_enable__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("robot_msgs.msg._dock_enable.DockEnable", full_classname_dest, 38) == 0);
  }
  robot_msgs__msg__DockEnable * ros_message = _ros_message;
  {  // dock1
    PyObject * field = PyObject_GetAttrString(_pymsg, "dock1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->dock1 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // dock2
    PyObject * field = PyObject_GetAttrString(_pymsg, "dock2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->dock2 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // dock3
    PyObject * field = PyObject_GetAttrString(_pymsg, "dock3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->dock3 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // dock4
    PyObject * field = PyObject_GetAttrString(_pymsg, "dock4");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->dock4 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_msgs__msg__dock_enable__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DockEnable */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_msgs.msg._dock_enable");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DockEnable");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_msgs__msg__DockEnable * ros_message = (robot_msgs__msg__DockEnable *)raw_ros_message;
  {  // dock1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->dock1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dock1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dock2
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->dock2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dock2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dock3
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->dock3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dock3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dock4
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->dock4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dock4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
