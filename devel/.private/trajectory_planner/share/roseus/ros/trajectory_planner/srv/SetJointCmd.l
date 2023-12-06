;; Auto-generated. Do not edit!


(when (boundp 'trajectory_planner::SetJointCmd)
  (if (not (find-package "TRAJECTORY_PLANNER"))
    (make-package "TRAJECTORY_PLANNER"))
  (shadow 'SetJointCmd (find-package "TRAJECTORY_PLANNER")))
(unless (find-package "TRAJECTORY_PLANNER::SETJOINTCMD")
  (make-package "TRAJECTORY_PLANNER::SETJOINTCMD"))
(unless (find-package "TRAJECTORY_PLANNER::SETJOINTCMDREQUEST")
  (make-package "TRAJECTORY_PLANNER::SETJOINTCMDREQUEST"))
(unless (find-package "TRAJECTORY_PLANNER::SETJOINTCMDRESPONSE")
  (make-package "TRAJECTORY_PLANNER::SETJOINTCMDRESPONSE"))

(in-package "ROS")





(defclass trajectory_planner::SetJointCmdRequest
  :super ros::object
  :slots (_jointAngle_1 _jointAngle_2 _jointAngle_3 _jointAngle_4 _jointAngle_5 _jointAngle_6 _speed ))

(defmethod trajectory_planner::SetJointCmdRequest
  (:init
   (&key
    ((:jointAngle_1 __jointAngle_1) 0.0)
    ((:jointAngle_2 __jointAngle_2) 0.0)
    ((:jointAngle_3 __jointAngle_3) 0.0)
    ((:jointAngle_4 __jointAngle_4) 0.0)
    ((:jointAngle_5 __jointAngle_5) 0.0)
    ((:jointAngle_6 __jointAngle_6) 0.0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _jointAngle_1 (float __jointAngle_1))
   (setq _jointAngle_2 (float __jointAngle_2))
   (setq _jointAngle_3 (float __jointAngle_3))
   (setq _jointAngle_4 (float __jointAngle_4))
   (setq _jointAngle_5 (float __jointAngle_5))
   (setq _jointAngle_6 (float __jointAngle_6))
   (setq _speed (round __speed))
   self)
  (:jointAngle_1
   (&optional __jointAngle_1)
   (if __jointAngle_1 (setq _jointAngle_1 __jointAngle_1)) _jointAngle_1)
  (:jointAngle_2
   (&optional __jointAngle_2)
   (if __jointAngle_2 (setq _jointAngle_2 __jointAngle_2)) _jointAngle_2)
  (:jointAngle_3
   (&optional __jointAngle_3)
   (if __jointAngle_3 (setq _jointAngle_3 __jointAngle_3)) _jointAngle_3)
  (:jointAngle_4
   (&optional __jointAngle_4)
   (if __jointAngle_4 (setq _jointAngle_4 __jointAngle_4)) _jointAngle_4)
  (:jointAngle_5
   (&optional __jointAngle_5)
   (if __jointAngle_5 (setq _jointAngle_5 __jointAngle_5)) _jointAngle_5)
  (:jointAngle_6
   (&optional __jointAngle_6)
   (if __jointAngle_6 (setq _jointAngle_6 __jointAngle_6)) _jointAngle_6)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _jointAngle_1
    4
    ;; float32 _jointAngle_2
    4
    ;; float32 _jointAngle_3
    4
    ;; float32 _jointAngle_4
    4
    ;; float32 _jointAngle_5
    4
    ;; float32 _jointAngle_6
    4
    ;; int32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _jointAngle_1
       (sys::poke _jointAngle_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _jointAngle_2
       (sys::poke _jointAngle_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _jointAngle_3
       (sys::poke _jointAngle_3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _jointAngle_4
       (sys::poke _jointAngle_4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _jointAngle_5
       (sys::poke _jointAngle_5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _jointAngle_6
       (sys::poke _jointAngle_6 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _speed
       (write-long _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _jointAngle_1
     (setq _jointAngle_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _jointAngle_2
     (setq _jointAngle_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _jointAngle_3
     (setq _jointAngle_3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _jointAngle_4
     (setq _jointAngle_4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _jointAngle_5
     (setq _jointAngle_5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _jointAngle_6
     (setq _jointAngle_6 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass trajectory_planner::SetJointCmdResponse
  :super ros::object
  :slots (_result ))

(defmethod trajectory_planner::SetJointCmdResponse
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

(defclass trajectory_planner::SetJointCmd
  :super ros::object
  :slots ())

(setf (get trajectory_planner::SetJointCmd :md5sum-) "b5f68cbcaa5e7a8e8c34ee9959541272")
(setf (get trajectory_planner::SetJointCmd :datatype-) "trajectory_planner/SetJointCmd")
(setf (get trajectory_planner::SetJointCmd :request) trajectory_planner::SetJointCmdRequest)
(setf (get trajectory_planner::SetJointCmd :response) trajectory_planner::SetJointCmdResponse)

(defmethod trajectory_planner::SetJointCmdRequest
  (:response () (instance trajectory_planner::SetJointCmdResponse :init)))

(setf (get trajectory_planner::SetJointCmdRequest :md5sum-) "b5f68cbcaa5e7a8e8c34ee9959541272")
(setf (get trajectory_planner::SetJointCmdRequest :datatype-) "trajectory_planner/SetJointCmdRequest")
(setf (get trajectory_planner::SetJointCmdRequest :definition-)
      "float32 jointAngle_1 
float32 jointAngle_2 
float32 jointAngle_3 
float32 jointAngle_4 
float32 jointAngle_5 
float32 jointAngle_6 
int32 speed 
---
int32 result
")

(setf (get trajectory_planner::SetJointCmdResponse :md5sum-) "b5f68cbcaa5e7a8e8c34ee9959541272")
(setf (get trajectory_planner::SetJointCmdResponse :datatype-) "trajectory_planner/SetJointCmdResponse")
(setf (get trajectory_planner::SetJointCmdResponse :definition-)
      "float32 jointAngle_1 
float32 jointAngle_2 
float32 jointAngle_3 
float32 jointAngle_4 
float32 jointAngle_5 
float32 jointAngle_6 
int32 speed 
---
int32 result
")



(provide :trajectory_planner/SetJointCmd "b5f68cbcaa5e7a8e8c34ee9959541272")


