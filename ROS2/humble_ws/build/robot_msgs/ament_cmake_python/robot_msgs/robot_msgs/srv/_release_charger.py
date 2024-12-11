# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:srv/ReleaseCharger.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReleaseCharger_Request(type):
    """Metaclass of message 'ReleaseCharger_Request'."""

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
                'robot_msgs.srv.ReleaseCharger_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__release_charger__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__release_charger__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__release_charger__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__release_charger__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__release_charger__request

            from std_msgs.msg import UInt16
            if UInt16.__class__._TYPE_SUPPORT is None:
                UInt16.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReleaseCharger_Request(metaclass=Metaclass_ReleaseCharger_Request):
    """Message class 'ReleaseCharger_Request'."""

    __slots__ = [
        '_id',
    ]

    _fields_and_field_types = {
        'id': 'std_msgs/UInt16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'UInt16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import UInt16
        self.id = kwargs.get('id', UInt16())

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
        if self.id != other.id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            from std_msgs.msg import UInt16
            assert \
                isinstance(value, UInt16), \
                "The 'id' field must be a sub message of type 'UInt16'"
        self._id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ReleaseCharger_Response(type):
    """Metaclass of message 'ReleaseCharger_Response'."""

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
                'robot_msgs.srv.ReleaseCharger_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__release_charger__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__release_charger__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__release_charger__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__release_charger__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__release_charger__response

            from std_msgs.msg import Bool
            if Bool.__class__._TYPE_SUPPORT is None:
                Bool.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReleaseCharger_Response(metaclass=Metaclass_ReleaseCharger_Response):
    """Message class 'ReleaseCharger_Response'."""

    __slots__ = [
        '_released',
    ]

    _fields_and_field_types = {
        'released': 'std_msgs/Bool',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Bool'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Bool
        self.released = kwargs.get('released', Bool())

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
        if self.released != other.released:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def released(self):
        """Message field 'released'."""
        return self._released

    @released.setter
    def released(self, value):
        if __debug__:
            from std_msgs.msg import Bool
            assert \
                isinstance(value, Bool), \
                "The 'released' field must be a sub message of type 'Bool'"
        self._released = value


class Metaclass_ReleaseCharger(type):
    """Metaclass of service 'ReleaseCharger'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_msgs.srv.ReleaseCharger')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__release_charger

            from robot_msgs.srv import _release_charger
            if _release_charger.Metaclass_ReleaseCharger_Request._TYPE_SUPPORT is None:
                _release_charger.Metaclass_ReleaseCharger_Request.__import_type_support__()
            if _release_charger.Metaclass_ReleaseCharger_Response._TYPE_SUPPORT is None:
                _release_charger.Metaclass_ReleaseCharger_Response.__import_type_support__()


class ReleaseCharger(metaclass=Metaclass_ReleaseCharger):
    from robot_msgs.srv._release_charger import ReleaseCharger_Request as Request
    from robot_msgs.srv._release_charger import ReleaseCharger_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
