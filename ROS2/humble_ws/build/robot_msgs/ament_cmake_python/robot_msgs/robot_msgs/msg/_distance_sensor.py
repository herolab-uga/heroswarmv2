# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:msg/DistanceSensor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DistanceSensor(type):
    """Metaclass of message 'DistanceSensor'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_msgs.msg.DistanceSensor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__distance_sensor
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__distance_sensor
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__distance_sensor
            cls._TYPE_SUPPORT = module.type_support_msg__msg__distance_sensor
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__distance_sensor

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DistanceSensor(metaclass=Metaclass_DistanceSensor):
    """Message class 'DistanceSensor'."""

    __slots__ = [
        '_sensor1',
        '_sensor2',
        '_sensor3',
        '_sensor4',
    ]

    _fields_and_field_types = {
        'sensor1': 'int32',
        'sensor2': 'int32',
        'sensor3': 'int32',
        'sensor4': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sensor1 = kwargs.get('sensor1', int())
        self.sensor2 = kwargs.get('sensor2', int())
        self.sensor3 = kwargs.get('sensor3', int())
        self.sensor4 = kwargs.get('sensor4', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.sensor1 != other.sensor1:
            return False
        if self.sensor2 != other.sensor2:
            return False
        if self.sensor3 != other.sensor3:
            return False
        if self.sensor4 != other.sensor4:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def sensor1(self):
        """Message field 'sensor1'."""
        return self._sensor1

    @sensor1.setter
    def sensor1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensor1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sensor1' field must be an integer in [-2147483648, 2147483647]"
        self._sensor1 = value

    @builtins.property
    def sensor2(self):
        """Message field 'sensor2'."""
        return self._sensor2

    @sensor2.setter
    def sensor2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensor2' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sensor2' field must be an integer in [-2147483648, 2147483647]"
        self._sensor2 = value

    @builtins.property
    def sensor3(self):
        """Message field 'sensor3'."""
        return self._sensor3

    @sensor3.setter
    def sensor3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensor3' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sensor3' field must be an integer in [-2147483648, 2147483647]"
        self._sensor3 = value

    @builtins.property
    def sensor4(self):
        """Message field 'sensor4'."""
        return self._sensor4

    @sensor4.setter
    def sensor4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensor4' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sensor4' field must be an integer in [-2147483648, 2147483647]"
        self._sensor4 = value
