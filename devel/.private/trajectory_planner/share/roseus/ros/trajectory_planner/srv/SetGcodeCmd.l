;; Auto-generated. Do not edit!


(when (boundp 'trajectory_planner::SetGcodeCmd)
  (if (not (find-package "TRAJECTORY_PLANNER"))
    (make-package "TRAJECTORY_PLANNER"))
  (shadow 'SetGcodeCmd (find-package "TRAJECTORY_PLANNER")))
(unless (find-package "TRAJECTORY_PLANNER::SETGCODECMD")
  (make-package "TRAJECTORY_PLANNER::SETGCODECMD"))
(unless (find-package "TRAJECTORY_PLANNER::SETGCODECMDREQUEST")
  (make-package "TRAJECTORY_PLANNER::SETGCODECMDREQUEST"))
(unless (find-package "TRAJECTORY_PLANNER::SETGCODECMDRESPONSE")
  (make-package "TRAJECTORY_PLANNER::SETGCODECMDRESPONSE"))

(in-package "ROS")





(defclass trajectory_planner::SetGcodeCmdRequest
  :super ros::object
  :slots (_gcode ))

(defmethod trajectory_planner::SetGcodeCmdRequest
  (:init
   (&key
    ((:gcode __gcode) "")
    )
   (send-super :init)
   (setq _gcode (string __gcode))
   self)
  (:gcode
   (&optional __gcode)
   (if __gcode (setq _gcode __gcode)) _gcode)
  (:serialization-length
   ()
   (+
    ;; string _gcode
    4 (length _gcode)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _gcode
       (write-long (length _gcode) s) (princ _gcode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _gcode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gcode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass trajectory_planner::SetGcodeCmdResponse
  :super ros::object
  :slots (_result ))

(defmethod trajectory_planner::SetGcodeCmdResponse
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

(defclass trajectory_planner::SetGcodeCmd
  :super ros::object
  :slots ())

(setf (get trajectory_planner::SetGcodeCmd :md5sum-) "063984922b4c75aaeda3d4d697e63bac")
(setf (get trajectory_planner::SetGcodeCmd :datatype-) "trajectory_planner/SetGcodeCmd")
(setf (get trajectory_planner::SetGcodeCmd :request) trajectory_planner::SetGcodeCmdRequest)
(setf (get trajectory_planner::SetGcodeCmd :response) trajectory_planner::SetGcodeCmdResponse)

(defmethod trajectory_planner::SetGcodeCmdRequest
  (:response () (instance trajectory_planner::SetGcodeCmdResponse :init)))

(setf (get trajectory_planner::SetGcodeCmdRequest :md5sum-) "063984922b4c75aaeda3d4d697e63bac")
(setf (get trajectory_planner::SetGcodeCmdRequest :datatype-) "trajectory_planner/SetGcodeCmdRequest")
(setf (get trajectory_planner::SetGcodeCmdRequest :definition-)
      "string gcode
---
int32 result
")

(setf (get trajectory_planner::SetGcodeCmdResponse :md5sum-) "063984922b4c75aaeda3d4d697e63bac")
(setf (get trajectory_planner::SetGcodeCmdResponse :datatype-) "trajectory_planner/SetGcodeCmdResponse")
(setf (get trajectory_planner::SetGcodeCmdResponse :definition-)
      "string gcode
---
int32 result
")



(provide :trajectory_planner/SetGcodeCmd "063984922b4c75aaeda3d4d697e63bac")


