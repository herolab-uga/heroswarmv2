;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::Environment)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'Environment (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::ENVIRONMENT")
  (make-package "ROBOT_MSGS::ENVIRONMENT"))

(in-package "ROS")
;;//! \htmlinclude Environment.msg.html


(defclass robot_msgs::Environment
  :super ros::object
  :slots (_temp _pressure _humidity _altitude ))

(defmethod robot_msgs::Environment
  (:init
   (&key
    ((:temp __temp) 0.0)
    ((:pressure __pressure) 0.0)
    ((:humidity __humidity) 0.0)
    ((:altitude __altitude) 0.0)
    )
   (send-super :init)
   (setq _temp (float __temp))
   (setq _pressure (float __pressure))
   (setq _humidity (float __humidity))
   (setq _altitude (float __altitude))
   self)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:pressure
   (&optional __pressure)
   (if __pressure (setq _pressure __pressure)) _pressure)
  (:humidity
   (&optional __humidity)
   (if __humidity (setq _humidity __humidity)) _humidity)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:serialization-length
   ()
   (+
    ;; float64 _temp
    8
    ;; float64 _pressure
    8
    ;; float64 _humidity
    8
    ;; float64 _altitude
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _temp
       (sys::poke _temp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pressure
       (sys::poke _pressure (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _humidity
       (sys::poke _humidity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _temp
     (setq _temp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pressure
     (setq _pressure (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _humidity
     (setq _humidity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get robot_msgs::Environment :md5sum-) "f1a388ffb9d45c754fa291d8040a937d")
(setf (get robot_msgs::Environment :datatype-) "robot_msgs/Environment")
(setf (get robot_msgs::Environment :definition-)
      "float64 temp
float64 pressure
float64 humidity
float64 altitude

")



(provide :robot_msgs/Environment "f1a388ffb9d45c754fa291d8040a937d")


