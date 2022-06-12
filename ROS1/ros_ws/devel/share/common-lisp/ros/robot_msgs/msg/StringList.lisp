; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude StringList.msg.html

(cl:defclass <StringList> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector std_msgs-msg:String)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:String :initial-element (cl:make-instance 'std_msgs-msg:String))))
)

(cl:defclass StringList (<StringList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StringList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StringList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<StringList> is deprecated: use robot_msgs-msg:StringList instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <StringList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:names-val is deprecated.  Use robot_msgs-msg:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StringList>) ostream)
  "Serializes a message object of type '<StringList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StringList>) istream)
  "Deserializes a message object of type '<StringList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:String))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StringList>)))
  "Returns string type for a message object of type '<StringList>"
  "robot_msgs/StringList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringList)))
  "Returns string type for a message object of type 'StringList"
  "robot_msgs/StringList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StringList>)))
  "Returns md5sum for a message object of type '<StringList>"
  "5334ccc36929d3443e2083f7204590bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StringList)))
  "Returns md5sum for a message object of type 'StringList"
  "5334ccc36929d3443e2083f7204590bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StringList>)))
  "Returns full string definition for message of type '<StringList>"
  (cl:format cl:nil "std_msgs/String[] names~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StringList)))
  "Returns full string definition for message of type 'StringList"
  (cl:format cl:nil "std_msgs/String[] names~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StringList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StringList>))
  "Converts a ROS message object to a list"
  (cl:list 'StringList
    (cl:cons ':names (names msg))
))
