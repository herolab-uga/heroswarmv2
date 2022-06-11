; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude Name.msg.html

(cl:defclass <Name> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (time
    :reader time
    :initarg :time
    :type std_msgs-msg:UInt64
    :initform (cl:make-instance 'std_msgs-msg:UInt64)))
)

(cl:defclass Name (<Name>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Name>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Name)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<Name> is deprecated: use robot_msgs-msg:Name instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Name>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:name-val is deprecated.  Use robot_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Name>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:time-val is deprecated.  Use robot_msgs-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Name>) ostream)
  "Serializes a message object of type '<Name>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Name>) istream)
  "Deserializes a message object of type '<Name>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Name>)))
  "Returns string type for a message object of type '<Name>"
  "robot_msgs/Name")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Name)))
  "Returns string type for a message object of type 'Name"
  "robot_msgs/Name")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Name>)))
  "Returns md5sum for a message object of type '<Name>"
  "f8b4d9de593bc07202bdba9902cdb0a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Name)))
  "Returns md5sum for a message object of type 'Name"
  "f8b4d9de593bc07202bdba9902cdb0a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Name>)))
  "Returns full string definition for message of type '<Name>"
  (cl:format cl:nil "std_msgs/String name~%std_msgs/UInt64 time~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt64~%uint64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Name)))
  "Returns full string definition for message of type 'Name"
  (cl:format cl:nil "std_msgs/String name~%std_msgs/UInt64 time~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt64~%uint64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Name>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Name>))
  "Converts a ROS message object to a list"
  (cl:list 'Name
    (cl:cons ':name (name msg))
    (cl:cons ':time (time msg))
))
