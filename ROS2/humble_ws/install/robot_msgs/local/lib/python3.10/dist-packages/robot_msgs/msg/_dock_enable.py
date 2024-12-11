# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:msg/DockEnable.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DockEnable(type):
    """Metaclass of message 'DockEnable'."""

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
                'robot_msgs.msg.DockEnable')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__dock_enable
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__dock_enable
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__dock_enable
            cls._TYPE_SUPPORT = module.type_support_msg__msg__dock_enable
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__dock_enable

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DockEnable(metaclass=Metaclass_DockEnable):
    """Message class 'DockEnable'."""

    __slots__ = [
        '_dock1',
        '_dock2',
        '_dock3',
        '_dock4',
    ]

    _fields_and_field_types = {
        'dock1': 'uint8',
        'dock2': 'uint8',
        'dock3': 'uint8',
        'dock4': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.dock1 = kwargs.get('dock1', int())
        self.dock2 = kwargs.get('dock2', int())
        self.dock3 = kwargs.get('dock3', int())
        self.dock4 = kwargs.get('dock4', int())

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
        if self.dock1 != other.dock1:
            return False
        if self.dock2 != other.dock2:
            return False
        if self.dock3 != other.dock3:
            return False
        if self.dock4 != other.dock4:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def dock1(self):
        """Message field 'dock1'."""
        return self._dock1

    @dock1.setter
    def dock1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'dock1' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'dock1' field must be an unsigned integer in [0, 255]"
        self._dock1 = value

    @builtins.property
    def dock2(self):
        """Message field 'dock2'."""
        return self._dock2

    @dock2.setter
    def dock2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'dock2' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'dock2' field must be an unsigned integer in [0, 255]"
        self._dock2 = value

    @builtins.property
    def dock3(self):
        """Message field 'dock3'."""
        return self._dock3

    @dock3.setter
    def dock3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'dock3' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'dock3' field must be an unsigned integer in [0, 255]"
        self._dock3 = value

    @builtins.property
    def dock4(self):
        """Message field 'dock4'."""
        return self._dock4

    @dock4.setter
    def dock4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'dock4' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'dock4' field must be an unsigned integer in [0, 255]"
        self._dock4 = value
