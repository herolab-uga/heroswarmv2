; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude NameList.msg.html

(cl:defclass <NameList> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector robot_msgs-msg:Name)
   :initform (cl:make-array 0 :element-type 'robot_msgs-msg:Name :initial-element (cl:make-instance 'robot_msgs-msg:Name))))
)

(cl:defclass NameList (<NameList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NameList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NameList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<NameList> is deprecated: use robot_msgs-msg:NameList instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <NameList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:names-val is deprecated.  Use robot_msgs-msg:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NameList>) ostream)
  "Serializes a message object of type '<NameList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NameList>) istream)
  "Deserializes a message object of type '<NameList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robot_msgs-msg:Name))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NameList>)))
  "Returns string type for a message object of type '<NameList>"
  "robot_msgs/NameList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NameList)))
  "Returns string type for a message object of type 'NameList"
  "robot_msgs/NameList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NameList>)))
  "Returns md5sum for a message object of type '<NameList>"
  "a56b0cece0aeebd6949775af9bc48613")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NameList)))
  "Returns md5sum for a message object of type 'NameList"
  "a56b0cece0aeebd6949775af9bc48613")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NameList>)))
  "Returns full string definition for message of type '<NameList>"
  (cl:format cl:nil "Name[] names~%================================================================================~%MSG: robot_msgs/Name~%std_msgs/String name~%std_msgs/UInt64 time~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt64~%uint64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NameList)))
  "Returns full string definition for message of type 'NameList"
  (cl:format cl:nil "Name[] names~%================================================================================~%MSG: robot_msgs/Name~%std_msgs/String name~%std_msgs/UInt64 time~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt64~%uint64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NameList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NameList>))
  "Converts a ROS message object to a list"
  (cl:list 'NameList
    (cl:cons ':names (names msg))
))
