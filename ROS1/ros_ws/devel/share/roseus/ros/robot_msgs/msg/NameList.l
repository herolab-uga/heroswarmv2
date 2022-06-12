;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::NameList)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'NameList (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::NAMELIST")
  (make-package "ROBOT_MSGS::NAMELIST"))

(in-package "ROS")
;;//! \htmlinclude NameList.msg.html


(defclass robot_msgs::NameList
  :super ros::object
  :slots (_names ))

(defmethod robot_msgs::NameList
  (:init
   (&key
    ((:names __names) ())
    )
   (send-super :init)
   (setq _names __names)
   self)
  (:names
   (&rest __names)
   (if (keywordp (car __names))
       (send* _names __names)
     (progn
       (if __names (setq _names (car __names)))
       _names)))
  (:serialization-length
   ()
   (+
    ;; robot_msgs/Name[] _names
    (apply #'+ (send-all _names :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_msgs/Name[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_msgs/Name[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (let (r) (dotimes (i n) (push (instance robot_msgs::Name :init) r)) r))
     (dolist (elem- _names)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get robot_msgs::NameList :md5sum-) "a56b0cece0aeebd6949775af9bc48613")
(setf (get robot_msgs::NameList :datatype-) "robot_msgs/NameList")
(setf (get robot_msgs::NameList :definition-)
      "Name[] names
================================================================================
MSG: robot_msgs/Name
std_msgs/String name
std_msgs/UInt64 time
================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/UInt64
uint64 data
")



(provide :robot_msgs/NameList "a56b0cece0aeebd6949775af9bc48613")


