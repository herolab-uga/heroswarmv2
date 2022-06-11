;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::StringList)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'StringList (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::STRINGLIST")
  (make-package "ROBOT_MSGS::STRINGLIST"))

(in-package "ROS")
;;//! \htmlinclude StringList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robot_msgs::StringList
  :super ros::object
  :slots (_names ))

(defmethod robot_msgs::StringList
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
    ;; std_msgs/String[] _names
    (apply #'+ (send-all _names :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (let (r) (dotimes (i n) (push (instance std_msgs::String :init) r)) r))
     (dolist (elem- _names)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get robot_msgs::StringList :md5sum-) "5334ccc36929d3443e2083f7204590bf")
(setf (get robot_msgs::StringList :datatype-) "robot_msgs/StringList")
(setf (get robot_msgs::StringList :definition-)
      "std_msgs/String[] names
================================================================================
MSG: std_msgs/String
string data

")



(provide :robot_msgs/StringList "5334ccc36929d3443e2083f7204590bf")


