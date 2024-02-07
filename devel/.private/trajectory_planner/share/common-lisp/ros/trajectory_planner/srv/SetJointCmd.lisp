; Auto-generated. Do not edit!


(cl:in-package trajectory_planner-srv)


;//! \htmlinclude SetJointCmd-request.msg.html

(cl:defclass <SetJointCmd-request> (roslisp-msg-protocol:ros-message)
  ((jointAngle_1
    :reader jointAngle_1
    :initarg :jointAngle_1
    :type cl:float
    :initform 0.0)
   (jointAngle_2
    :reader jointAngle_2
    :initarg :jointAngle_2
    :type cl:float
    :initform 0.0)
   (jointAngle_3
    :reader jointAngle_3
    :initarg :jointAngle_3
    :type cl:float
    :initform 0.0)
   (jointAngle_4
    :reader jointAngle_4
    :initarg :jointAngle_4
    :type cl:float
    :initform 0.0)
   (jointAngle_5
    :reader jointAngle_5
    :initarg :jointAngle_5
    :type cl:float
    :initform 0.0)
   (jointAngle_6
    :reader jointAngle_6
    :initarg :jointAngle_6
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass SetJointCmd-request (<SetJointCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<SetJointCmd-request> is deprecated: use trajectory_planner-srv:SetJointCmd-request instead.")))

(cl:ensure-generic-function 'jointAngle_1-val :lambda-list '(m))
(cl:defmethod jointAngle_1-val ((m <SetJointCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_1-val is deprecated.  Use trajectory_planner-srv:jointAngle_1 instead.")
  (jointAngle_1 m))

(cl:ensure-generic-function 'jointAngle_2-val :lambda-list '(m))
(cl:defmethod jointAngle_2-val ((m <SetJointCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_2-val is deprecated.  Use trajectory_planner-srv:jointAngle_2 instead.")
  (jointAngle_2 m))

(cl:ensure-generic-function 'jointAngle_3-val :lambda-list '(m))
(cl:defmethod jointAngle_3-val ((m <SetJointCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_3-val is deprecated.  Use trajectory_planner-srv:jointAngle_3 instead.")
  (jointAngle_3 m))

(cl:ensure-generic-function 'jointAngle_4-val :lambda-list '(m))
(cl:defmethod jointAngle_4-val ((m <SetJointCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_4-val is deprecated.  Use trajectory_planner-srv:jointAngle_4 instead.")
  (jointAngle_4 m))

(cl:ensure-generic-function 'jointAngle_5-val :lambda-list '(m))
(cl:defmethod jointAngle_5-val ((m <SetJointCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_5-val is deprecated.  Use trajectory_planner-srv:jointAngle_5 instead.")
  (jointAngle_5 m))

(cl:ensure-generic-function 'jointAngle_6-val :lambda-list '(m))
(cl:defmethod jointAngle_6-val ((m <SetJointCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_6-val is deprecated.  Use trajectory_planner-srv:jointAngle_6 instead.")
  (jointAngle_6 m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <SetJointCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:speed-val is deprecated.  Use trajectory_planner-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointCmd-request>) ostream)
  "Serializes a message object of type '<SetJointCmd-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jointAngle_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jointAngle_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jointAngle_3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jointAngle_4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jointAngle_5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jointAngle_6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointCmd-request>) istream)
  "Deserializes a message object of type '<SetJointCmd-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointAngle_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointAngle_2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointAngle_3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointAngle_4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointAngle_5) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointAngle_6) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointCmd-request>)))
  "Returns string type for a service object of type '<SetJointCmd-request>"
  "trajectory_planner/SetJointCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointCmd-request)))
  "Returns string type for a service object of type 'SetJointCmd-request"
  "trajectory_planner/SetJointCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointCmd-request>)))
  "Returns md5sum for a message object of type '<SetJointCmd-request>"
  "b5f68cbcaa5e7a8e8c34ee9959541272")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointCmd-request)))
  "Returns md5sum for a message object of type 'SetJointCmd-request"
  "b5f68cbcaa5e7a8e8c34ee9959541272")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointCmd-request>)))
  "Returns full string definition for message of type '<SetJointCmd-request>"
  (cl:format cl:nil "float32 jointAngle_1 ~%float32 jointAngle_2 ~%float32 jointAngle_3 ~%float32 jointAngle_4 ~%float32 jointAngle_5 ~%float32 jointAngle_6 ~%int32 speed ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointCmd-request)))
  "Returns full string definition for message of type 'SetJointCmd-request"
  (cl:format cl:nil "float32 jointAngle_1 ~%float32 jointAngle_2 ~%float32 jointAngle_3 ~%float32 jointAngle_4 ~%float32 jointAngle_5 ~%float32 jointAngle_6 ~%int32 speed ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointCmd-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointCmd-request
    (cl:cons ':jointAngle_1 (jointAngle_1 msg))
    (cl:cons ':jointAngle_2 (jointAngle_2 msg))
    (cl:cons ':jointAngle_3 (jointAngle_3 msg))
    (cl:cons ':jointAngle_4 (jointAngle_4 msg))
    (cl:cons ':jointAngle_5 (jointAngle_5 msg))
    (cl:cons ':jointAngle_6 (jointAngle_6 msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude SetJointCmd-response.msg.html

(cl:defclass <SetJointCmd-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass SetJointCmd-response (<SetJointCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<SetJointCmd-response> is deprecated: use trajectory_planner-srv:SetJointCmd-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetJointCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:result-val is deprecated.  Use trajectory_planner-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointCmd-response>) ostream)
  "Serializes a message object of type '<SetJointCmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointCmd-response>) istream)
  "Deserializes a message object of type '<SetJointCmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointCmd-response>)))
  "Returns string type for a service object of type '<SetJointCmd-response>"
  "trajectory_planner/SetJointCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointCmd-response)))
  "Returns string type for a service object of type 'SetJointCmd-response"
  "trajectory_planner/SetJointCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointCmd-response>)))
  "Returns md5sum for a message object of type '<SetJointCmd-response>"
  "b5f68cbcaa5e7a8e8c34ee9959541272")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointCmd-response)))
  "Returns md5sum for a message object of type 'SetJointCmd-response"
  "b5f68cbcaa5e7a8e8c34ee9959541272")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointCmd-response>)))
  "Returns full string definition for message of type '<SetJointCmd-response>"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointCmd-response)))
  "Returns full string definition for message of type 'SetJointCmd-response"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointCmd-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointCmd-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointCmd)))
  'SetJointCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointCmd)))
  'SetJointCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointCmd)))
  "Returns string type for a service object of type '<SetJointCmd>"
  "trajectory_planner/SetJointCmd")