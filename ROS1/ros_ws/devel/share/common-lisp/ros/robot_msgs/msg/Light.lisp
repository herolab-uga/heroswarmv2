; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude Light.msg.html

(cl:defclass <Light> (roslisp-msg-protocol:ros-message)
  ((rgbw
    :reader rgbw
    :initarg :rgbw
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (gesture
    :reader gesture
    :initarg :gesture
    :type cl:integer
    :initform 0))
)

(cl:defclass Light (<Light>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Light>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Light)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<Light> is deprecated: use robot_msgs-msg:Light instead.")))

(cl:ensure-generic-function 'rgbw-val :lambda-list '(m))
(cl:defmethod rgbw-val ((m <Light>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:rgbw-val is deprecated.  Use robot_msgs-msg:rgbw instead.")
  (rgbw m))

(cl:ensure-generic-function 'gesture-val :lambda-list '(m))
(cl:defmethod gesture-val ((m <Light>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:gesture-val is deprecated.  Use robot_msgs-msg:gesture instead.")
  (gesture m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Light>) ostream)
  "Serializes a message object of type '<Light>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rgbw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'rgbw))
  (cl:let* ((signed (cl:slot-value msg 'gesture)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Light>) istream)
  "Deserializes a message object of type '<Light>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rgbw) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rgbw)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gesture) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Light>)))
  "Returns string type for a message object of type '<Light>"
  "robot_msgs/Light")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Light)))
  "Returns string type for a message object of type 'Light"
  "robot_msgs/Light")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Light>)))
  "Returns md5sum for a message object of type '<Light>"
  "ba54a502272d86cc6063f7ba6b342cf5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Light)))
  "Returns md5sum for a message object of type 'Light"
  "ba54a502272d86cc6063f7ba6b342cf5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Light>)))
  "Returns full string definition for message of type '<Light>"
  (cl:format cl:nil "int32[] rgbw~%int32 gesture~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Light)))
  "Returns full string definition for message of type 'Light"
  (cl:format cl:nil "int32[] rgbw~%int32 gesture~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Light>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rgbw) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Light>))
  "Converts a ROS message object to a list"
  (cl:list 'Light
    (cl:cons ':rgbw (rgbw msg))
    (cl:cons ':gesture (gesture msg))
))
