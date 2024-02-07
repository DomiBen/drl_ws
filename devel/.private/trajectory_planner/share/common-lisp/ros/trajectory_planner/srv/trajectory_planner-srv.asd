
(cl:in-package :asdf)

(defsystem "trajectory_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetPoseCmd" :depends-on ("_package_GetPoseCmd"))
    (:file "_package_GetPoseCmd" :depends-on ("_package"))
    (:file "SetCartCmd" :depends-on ("_package_SetCartCmd"))
    (:file "_package_SetCartCmd" :depends-on ("_package"))
    (:file "SetGcodeCmd" :depends-on ("_package_SetGcodeCmd"))
    (:file "_package_SetGcodeCmd" :depends-on ("_package"))
    (:file "SetHomeCmd" :depends-on ("_package_SetHomeCmd"))
    (:file "_package_SetHomeCmd" :depends-on ("_package"))
    (:file "SetJointCmd" :depends-on ("_package_SetJointCmd"))
    (:file "_package_SetJointCmd" :depends-on ("_package"))
  ))