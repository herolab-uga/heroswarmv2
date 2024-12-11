# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:srv/GetCharger.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetCharger_Request(type):
    """Metaclass of message 'GetCharger_Request'."""

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
                'robot_msgs.srv.GetCharger_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_charger__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_charger__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_charger__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_charger__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_charger__request

            from std_msgs.msg import String
            if String.__class__._TYPE_SUPPORT is None:
                String.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetCharger_Request(metaclass=Metaclass_GetCharger_Request):
    """Message class 'GetCharger_Request'."""

    __slots__ = [
        '_name',
    ]

    _fields_and_field_types = {
        'name': 'std_msgs/String',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import String
        self.name = kwargs.get('name', String())

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
        if self.name != other.name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'name' field must be a sub message of type 'String'"
        self._name = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetCharger_Response(type):
    """Metaclass of message 'GetCharger_Response'."""

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
                'robot_msgs.srv.GetCharger_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_charger__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_charger__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_charger__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_charger__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_charger__response

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

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


class GetCharger_Response(metaclass=Metaclass_GetCharger_Response):
    """Message class 'GetCharger_Response'."""

    __slots__ = [
        '_id',
        '_position',
    ]

    _fields_and_field_types = {
        'id': 'std_msgs/UInt16',
        'position': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'UInt16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import UInt16
        self.id = kwargs.get('id', UInt16())
        from geometry_msgs.msg import Pose
        self.position = kwargs.get('position', Pose())

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
        if self.position != other.position:
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

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'position' field must be a sub message of type 'Pose'"
        self._position = value


class Metaclass_GetCharger(type):
    """Metaclass of service 'GetCharger'."""

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
                'robot_msgs.srv.GetCharger')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_charger

            from robot_msgs.srv import _get_charger
            if _get_charger.Metaclass_GetCharger_Request._TYPE_SUPPORT is None:
                _get_charger.Metaclass_GetCharger_Request.__import_type_support__()
            if _get_charger.Metaclass_GetCharger_Response._TYPE_SUPPORT is None:
                _get_charger.Metaclass_GetCharger_Response.__import_type_support__()


class GetCharger(metaclass=Metaclass_GetCharger):
    from robot_msgs.srv._get_charger import GetCharger_Request as Request
    from robot_msgs.srv._get_charger import GetCharger_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
