; Auto-generated. Do not edit!


(cl:in-package trajectory_planner-srv)


;//! \htmlinclude GetPoseCmd-request.msg.html

(cl:defclass <GetPoseCmd-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPoseCmd-request (<GetPoseCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPoseCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPoseCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<GetPoseCmd-request> is deprecated: use trajectory_planner-srv:GetPoseCmd-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPoseCmd-request>) ostream)
  "Serializes a message object of type '<GetPoseCmd-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPoseCmd-request>) istream)
  "Deserializes a message object of type '<GetPoseCmd-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPoseCmd-request>)))
  "Returns string type for a service object of type '<GetPoseCmd-request>"
  "trajectory_planner/GetPoseCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPoseCmd-request)))
  "Returns string type for a service object of type 'GetPoseCmd-request"
  "trajectory_planner/GetPoseCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPoseCmd-request>)))
  "Returns md5sum for a message object of type '<GetPoseCmd-request>"
  "5aef3792e74d728815d4dfbf8b1f490f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPoseCmd-request)))
  "Returns md5sum for a message object of type 'GetPoseCmd-request"
  "5aef3792e74d728815d4dfbf8b1f490f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPoseCmd-request>)))
  "Returns full string definition for message of type '<GetPoseCmd-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPoseCmd-request)))
  "Returns full string definition for message of type 'GetPoseCmd-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPoseCmd-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPoseCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPoseCmd-request
))
;//! \htmlinclude GetPoseCmd-response.msg.html

(cl:defclass <GetPoseCmd-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (b
    :reader b
    :initarg :b
    :type cl:float
    :initform 0.0)
   (c
    :reader c
    :initarg :c
    :type cl:float
    :initform 0.0)
   (jointAngle_1
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
   (jointAngle_7
    :reader jointAngle_7
    :initarg :jointAngle_7
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetPoseCmd-response (<GetPoseCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPoseCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPoseCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<GetPoseCmd-response> is deprecated: use trajectory_planner-srv:GetPoseCmd-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:result-val is deprecated.  Use trajectory_planner-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:state-val is deprecated.  Use trajectory_planner-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:x-val is deprecated.  Use trajectory_planner-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:y-val is deprecated.  Use trajectory_planner-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:z-val is deprecated.  Use trajectory_planner-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:a-val is deprecated.  Use trajectory_planner-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:b-val is deprecated.  Use trajectory_planner-srv:b instead.")
  (b m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:c-val is deprecated.  Use trajectory_planner-srv:c instead.")
  (c m))

(cl:ensure-generic-function 'jointAngle_1-val :lambda-list '(m))
(cl:defmethod jointAngle_1-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_1-val is deprecated.  Use trajectory_planner-srv:jointAngle_1 instead.")
  (jointAngle_1 m))

(cl:ensure-generic-function 'jointAngle_2-val :lambda-list '(m))
(cl:defmethod jointAngle_2-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_2-val is deprecated.  Use trajectory_planner-srv:jointAngle_2 instead.")
  (jointAngle_2 m))

(cl:ensure-generic-function 'jointAngle_3-val :lambda-list '(m))
(cl:defmethod jointAngle_3-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_3-val is deprecated.  Use trajectory_planner-srv:jointAngle_3 instead.")
  (jointAngle_3 m))

(cl:ensure-generic-function 'jointAngle_4-val :lambda-list '(m))
(cl:defmethod jointAngle_4-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_4-val is deprecated.  Use trajectory_planner-srv:jointAngle_4 instead.")
  (jointAngle_4 m))

(cl:ensure-generic-function 'jointAngle_5-val :lambda-list '(m))
(cl:defmethod jointAngle_5-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_5-val is deprecated.  Use trajectory_planner-srv:jointAngle_5 instead.")
  (jointAngle_5 m))

(cl:ensure-generic-function 'jointAngle_6-val :lambda-list '(m))
(cl:defmethod jointAngle_6-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_6-val is deprecated.  Use trajectory_planner-srv:jointAngle_6 instead.")
  (jointAngle_6 m))

(cl:ensure-generic-function 'jointAngle_7-val :lambda-list '(m))
(cl:defmethod jointAngle_7-val ((m <GetPoseCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:jointAngle_7-val is deprecated.  Use trajectory_planner-srv:jointAngle_7 instead.")
  (jointAngle_7 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPoseCmd-response>) ostream)
  "Serializes a message object of type '<GetPoseCmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jointAngle_7))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPoseCmd-response>) istream)
  "Deserializes a message object of type '<GetPoseCmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointAngle_7) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPoseCmd-response>)))
  "Returns string type for a service object of type '<GetPoseCmd-response>"
  "trajectory_planner/GetPoseCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPoseCmd-response)))
  "Returns string type for a service object of type 'GetPoseCmd-response"
  "trajectory_planner/GetPoseCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPoseCmd-response>)))
  "Returns md5sum for a message object of type '<GetPoseCmd-response>"
  "5aef3792e74d728815d4dfbf8b1f490f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPoseCmd-response)))
  "Returns md5sum for a message object of type 'GetPoseCmd-response"
  "5aef3792e74d728815d4dfbf8b1f490f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPoseCmd-response>)))
  "Returns full string definition for message of type '<GetPoseCmd-response>"
  (cl:format cl:nil "int32 result~%int32 state~%float32 x~%float32 y~%float32 z~%float32 a~%float32 b~%float32 c~%float32 jointAngle_1~%float32 jointAngle_2~%float32 jointAngle_3~%float32 jointAngle_4~%float32 jointAngle_5~%float32 jointAngle_6~%float32 jointAngle_7~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPoseCmd-response)))
  "Returns full string definition for message of type 'GetPoseCmd-response"
  (cl:format cl:nil "int32 result~%int32 state~%float32 x~%float32 y~%float32 z~%float32 a~%float32 b~%float32 c~%float32 jointAngle_1~%float32 jointAngle_2~%float32 jointAngle_3~%float32 jointAngle_4~%float32 jointAngle_5~%float32 jointAngle_6~%float32 jointAngle_7~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPoseCmd-response>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPoseCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPoseCmd-response
    (cl:cons ':result (result msg))
    (cl:cons ':state (state msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
    (cl:cons ':c (c msg))
    (cl:cons ':jointAngle_1 (jointAngle_1 msg))
    (cl:cons ':jointAngle_2 (jointAngle_2 msg))
    (cl:cons ':jointAngle_3 (jointAngle_3 msg))
    (cl:cons ':jointAngle_4 (jointAngle_4 msg))
    (cl:cons ':jointAngle_5 (jointAngle_5 msg))
    (cl:cons ':jointAngle_6 (jointAngle_6 msg))
    (cl:cons ':jointAngle_7 (jointAngle_7 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPoseCmd)))
  'GetPoseCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPoseCmd)))
  'GetPoseCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPoseCmd)))
  "Returns string type for a service object of type '<GetPoseCmd>"
  "trajectory_planner/GetPoseCmd")