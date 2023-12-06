; Auto-generated. Do not edit!


(cl:in-package trajectory_planner-srv)


;//! \htmlinclude SetHomeCmd-request.msg.html

(cl:defclass <SetHomeCmd-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetHomeCmd-request (<SetHomeCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHomeCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHomeCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<SetHomeCmd-request> is deprecated: use trajectory_planner-srv:SetHomeCmd-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHomeCmd-request>) ostream)
  "Serializes a message object of type '<SetHomeCmd-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHomeCmd-request>) istream)
  "Deserializes a message object of type '<SetHomeCmd-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHomeCmd-request>)))
  "Returns string type for a service object of type '<SetHomeCmd-request>"
  "trajectory_planner/SetHomeCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHomeCmd-request)))
  "Returns string type for a service object of type 'SetHomeCmd-request"
  "trajectory_planner/SetHomeCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHomeCmd-request>)))
  "Returns md5sum for a message object of type '<SetHomeCmd-request>"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHomeCmd-request)))
  "Returns md5sum for a message object of type 'SetHomeCmd-request"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHomeCmd-request>)))
  "Returns full string definition for message of type '<SetHomeCmd-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHomeCmd-request)))
  "Returns full string definition for message of type 'SetHomeCmd-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHomeCmd-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHomeCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHomeCmd-request
))
;//! \htmlinclude SetHomeCmd-response.msg.html

(cl:defclass <SetHomeCmd-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass SetHomeCmd-response (<SetHomeCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetHomeCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetHomeCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<SetHomeCmd-response> is deprecated: use trajectory_planner-srv:SetHomeCmd-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetHomeCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:result-val is deprecated.  Use trajectory_planner-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetHomeCmd-response>) ostream)
  "Serializes a message object of type '<SetHomeCmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetHomeCmd-response>) istream)
  "Deserializes a message object of type '<SetHomeCmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetHomeCmd-response>)))
  "Returns string type for a service object of type '<SetHomeCmd-response>"
  "trajectory_planner/SetHomeCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHomeCmd-response)))
  "Returns string type for a service object of type 'SetHomeCmd-response"
  "trajectory_planner/SetHomeCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetHomeCmd-response>)))
  "Returns md5sum for a message object of type '<SetHomeCmd-response>"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetHomeCmd-response)))
  "Returns md5sum for a message object of type 'SetHomeCmd-response"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetHomeCmd-response>)))
  "Returns full string definition for message of type '<SetHomeCmd-response>"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetHomeCmd-response)))
  "Returns full string definition for message of type 'SetHomeCmd-response"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetHomeCmd-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetHomeCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetHomeCmd-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetHomeCmd)))
  'SetHomeCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetHomeCmd)))
  'SetHomeCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetHomeCmd)))
  "Returns string type for a service object of type '<SetHomeCmd>"
  "trajectory_planner/SetHomeCmd")