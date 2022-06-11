
(cl:in-package :asdf)

(defsystem "robot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Environment" :depends-on ("_package_Environment"))
    (:file "_package_Environment" :depends-on ("_package"))
    (:file "Light" :depends-on ("_package_Light"))
    (:file "_package_Light" :depends-on ("_package"))
    (:file "Robot_Pos" :depends-on ("_package_Robot_Pos"))
    (:file "_package_Robot_Pos" :depends-on ("_package"))
    (:file "StringList" :depends-on ("_package_StringList"))
    (:file "_package_StringList" :depends-on ("_package"))
  ))