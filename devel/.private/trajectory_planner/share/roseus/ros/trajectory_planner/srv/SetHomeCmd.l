;; Auto-generated. Do not edit!


(when (boundp 'trajectory_planner::SetHomeCmd)
  (if (not (find-package "TRAJECTORY_PLANNER"))
    (make-package "TRAJECTORY_PLANNER"))
  (shadow 'SetHomeCmd (find-package "TRAJECTORY_PLANNER")))
(unless (find-package "TRAJECTORY_PLANNER::SETHOMECMD")
  (make-package "TRAJECTORY_PLANNER::SETHOMECMD"))
(unless (find-package "TRAJECTORY_PLANNER::SETHOMECMDREQUEST")
  (make-package "TRAJECTORY_PLANNER::SETHOMECMDREQUEST"))
(unless (find-package "TRAJECTORY_PLANNER::SETHOMECMDRESPONSE")
  (make-package "TRAJECTORY_PLANNER::SETHOMECMDRESPONSE"))

(in-package "ROS")





(defclass trajectory_planner::SetHomeCmdRequest
  :super ros::object
  :slots ())

(defmethod trajectory_planner::SetHomeCmdRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass trajectory_planner::SetHomeCmdResponse
  :super ros::object
  :slots (_result ))

(defmethod trajectory_planner::SetHomeCmdResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass trajectory_planner::SetHomeCmd
  :super ros::object
  :slots ())

(setf (get trajectory_planner::SetHomeCmd :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get trajectory_planner::SetHomeCmd :datatype-) "trajectory_planner/SetHomeCmd")
(setf (get trajectory_planner::SetHomeCmd :request) trajectory_planner::SetHomeCmdRequest)
(setf (get trajectory_planner::SetHomeCmd :response) trajectory_planner::SetHomeCmdResponse)

(defmethod trajectory_planner::SetHomeCmdRequest
  (:response () (instance trajectory_planner::SetHomeCmdResponse :init)))

(setf (get trajectory_planner::SetHomeCmdRequest :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get trajectory_planner::SetHomeCmdRequest :datatype-) "trajectory_planner/SetHomeCmdRequest")
(setf (get trajectory_planner::SetHomeCmdRequest :definition-)
      "---
int32 result
")

(setf (get trajectory_planner::SetHomeCmdResponse :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get trajectory_planner::SetHomeCmdResponse :datatype-) "trajectory_planner/SetHomeCmdResponse")
(setf (get trajectory_planner::SetHomeCmdResponse :definition-)
      "---
int32 result
")



(provide :trajectory_planner/SetHomeCmd "034a8e20d6a306665e3a5b340fab3f09")


