;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::Robot_Pos)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'Robot_Pos (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::ROBOT_POS")
  (make-package "ROBOT_MSGS::ROBOT_POS"))

(in-package "ROS")
;;//! \htmlinclude Robot_Pos.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass robot_msgs::Robot_Pos
  :super ros::object
  :slots (_robot_pos ))

(defmethod robot_msgs::Robot_Pos
  (:init
   (&key
    ((:robot_pos __robot_pos) ())
    )
   (send-super :init)
   (setq _robot_pos __robot_pos)
   self)
  (:robot_pos
   (&rest __robot_pos)
   (if (keywordp (car __robot_pos))
       (send* _robot_pos __robot_pos)
     (progn
       (if __robot_pos (setq _robot_pos (car __robot_pos)))
       _robot_pos)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Odometry[] _robot_pos
    (apply #'+ (send-all _robot_pos :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Odometry[] _robot_pos
     (write-long (length _robot_pos) s)
     (dolist (elem _robot_pos)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Odometry[] _robot_pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_pos (let (r) (dotimes (i n) (push (instance nav_msgs::Odometry :init) r)) r))
     (dolist (elem- _robot_pos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get robot_msgs::Robot_Pos :md5sum-) "5feb85af36036d030a77b0accd3f21f1")
(setf (get robot_msgs::Robot_Pos :datatype-) "robot_msgs/Robot_Pos")
(setf (get robot_msgs::Robot_Pos :definition-)
      "nav_msgs/Odometry[] robot_pos
================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :robot_msgs/Robot_Pos "5feb85af36036d030a77b0accd3f21f1")


