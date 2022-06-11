;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::Light)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'Light (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::LIGHT")
  (make-package "ROBOT_MSGS::LIGHT"))

(in-package "ROS")
;;//! \htmlinclude Light.msg.html


(defclass robot_msgs::Light
  :super ros::object
  :slots (_rgbw _gesture ))

(defmethod robot_msgs::Light
  (:init
   (&key
    ((:rgbw __rgbw) (make-array 0 :initial-element 0 :element-type :integer))
    ((:gesture __gesture) 0)
    )
   (send-super :init)
   (setq _rgbw __rgbw)
   (setq _gesture (round __gesture))
   self)
  (:rgbw
   (&optional __rgbw)
   (if __rgbw (setq _rgbw __rgbw)) _rgbw)
  (:gesture
   (&optional __gesture)
   (if __gesture (setq _gesture __gesture)) _gesture)
  (:serialization-length
   ()
   (+
    ;; int32[] _rgbw
    (* 4    (length _rgbw)) 4
    ;; int32 _gesture
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _rgbw
     (write-long (length _rgbw) s)
     (dotimes (i (length _rgbw))
       (write-long (elt _rgbw i) s)
       )
     ;; int32 _gesture
       (write-long _gesture s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _rgbw
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rgbw (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _rgbw i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _gesture
     (setq _gesture (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_msgs::Light :md5sum-) "ba54a502272d86cc6063f7ba6b342cf5")
(setf (get robot_msgs::Light :datatype-) "robot_msgs/Light")
(setf (get robot_msgs::Light :definition-)
      "int32[] rgbw
int32 gesture
")



(provide :robot_msgs/Light "ba54a502272d86cc6063f7ba6b342cf5")


