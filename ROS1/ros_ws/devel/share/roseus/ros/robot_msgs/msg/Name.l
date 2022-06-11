;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::Name)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'Name (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::NAME")
  (make-package "ROBOT_MSGS::NAME"))

(in-package "ROS")
;;//! \htmlinclude Name.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robot_msgs::Name
  :super ros::object
  :slots (_name _time ))

(defmethod robot_msgs::Name
  (:init
   (&key
    ((:name __name) (instance std_msgs::String :init))
    ((:time __time) (instance std_msgs::UInt64 :init))
    )
   (send-super :init)
   (setq _name __name)
   (setq _time __time)
   self)
  (:name
   (&rest __name)
   (if (keywordp (car __name))
       (send* _name __name)
     (progn
       (if __name (setq _name (car __name)))
       _name)))
  (:time
   (&rest __time)
   (if (keywordp (car __time))
       (send* _time __time)
     (progn
       (if __time (setq _time (car __time)))
       _time)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _name
    (send _name :serialization-length)
    ;; std_msgs/UInt64 _time
    (send _time :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _name
       (send _name :serialize s)
     ;; std_msgs/UInt64 _time
       (send _time :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _name
     (send _name :deserialize buf ptr-) (incf ptr- (send _name :serialization-length))
   ;; std_msgs/UInt64 _time
     (send _time :deserialize buf ptr-) (incf ptr- (send _time :serialization-length))
   ;;
   self)
  )

(setf (get robot_msgs::Name :md5sum-) "f8b4d9de593bc07202bdba9902cdb0a6")
(setf (get robot_msgs::Name :datatype-) "robot_msgs/Name")
(setf (get robot_msgs::Name :definition-)
      "std_msgs/String name
std_msgs/UInt64 time
================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/UInt64
uint64 data
")



(provide :robot_msgs/Name "f8b4d9de593bc07202bdba9902cdb0a6")


