# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:msg/SensorEnable.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SensorEnable(type):
    """Metaclass of message 'SensorEnable'."""

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
                'robot_msgs.msg.SensorEnable')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sensor_enable
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sensor_enable
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sensor_enable
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sensor_enable
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sensor_enable

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SensorEnable(metaclass=Metaclass_SensorEnable):
    """Message class 'SensorEnable'."""

    __slots__ = [
        '_environment',
        '_imu',
        '_light',
        '_proximity',
        '_mic',
    ]

    _fields_and_field_types = {
        'environment': 'boolean',
        'imu': 'boolean',
        'light': 'boolean',
        'proximity': 'boolean',
        'mic': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.environment = kwargs.get('environment', bool())
        self.imu = kwargs.get('imu', bool())
        self.light = kwargs.get('light', bool())
        self.proximity = kwargs.get('proximity', bool())
        self.mic = kwargs.get('mic', bool())

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
        if self.environment != other.environment:
            return False
        if self.imu != other.imu:
            return False
        if self.light != other.light:
            return False
        if self.proximity != other.proximity:
            return False
        if self.mic != other.mic:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def environment(self):
        """Message field 'environment'."""
        return self._environment

    @environment.setter
    def environment(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'environment' field must be of type 'bool'"
        self._environment = value

    @builtins.property
    def imu(self):
        """Message field 'imu'."""
        return self._imu

    @imu.setter
    def imu(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'imu' field must be of type 'bool'"
        self._imu = value

    @builtins.property
    def light(self):
        """Message field 'light'."""
        return self._light

    @light.setter
    def light(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'light' field must be of type 'bool'"
        self._light = value

    @builtins.property
    def proximity(self):
        """Message field 'proximity'."""
        return self._proximity

    @proximity.setter
    def proximity(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'proximity' field must be of type 'bool'"
        self._proximity = value

    @builtins.property
    def mic(self):
        """Message field 'mic'."""
        return self._mic

    @mic.setter
    def mic(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'mic' field must be of type 'bool'"
        self._mic = value
