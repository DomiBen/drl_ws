; Auto-generated. Do not edit!


(cl:in-package trajectory_planner-srv)


;//! \htmlinclude SetGcodeCmd-request.msg.html

(cl:defclass <SetGcodeCmd-request> (roslisp-msg-protocol:ros-message)
  ((gcode
    :reader gcode
    :initarg :gcode
    :type cl:string
    :initform ""))
)

(cl:defclass SetGcodeCmd-request (<SetGcodeCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGcodeCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGcodeCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<SetGcodeCmd-request> is deprecated: use trajectory_planner-srv:SetGcodeCmd-request instead.")))

(cl:ensure-generic-function 'gcode-val :lambda-list '(m))
(cl:defmethod gcode-val ((m <SetGcodeCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:gcode-val is deprecated.  Use trajectory_planner-srv:gcode instead.")
  (gcode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGcodeCmd-request>) ostream)
  "Serializes a message object of type '<SetGcodeCmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gcode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gcode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGcodeCmd-request>) istream)
  "Deserializes a message object of type '<SetGcodeCmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gcode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gcode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGcodeCmd-request>)))
  "Returns string type for a service object of type '<SetGcodeCmd-request>"
  "trajectory_planner/SetGcodeCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGcodeCmd-request)))
  "Returns string type for a service object of type 'SetGcodeCmd-request"
  "trajectory_planner/SetGcodeCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGcodeCmd-request>)))
  "Returns md5sum for a message object of type '<SetGcodeCmd-request>"
  "063984922b4c75aaeda3d4d697e63bac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGcodeCmd-request)))
  "Returns md5sum for a message object of type 'SetGcodeCmd-request"
  "063984922b4c75aaeda3d4d697e63bac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGcodeCmd-request>)))
  "Returns full string definition for message of type '<SetGcodeCmd-request>"
  (cl:format cl:nil "string gcode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGcodeCmd-request)))
  "Returns full string definition for message of type 'SetGcodeCmd-request"
  (cl:format cl:nil "string gcode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGcodeCmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'gcode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGcodeCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGcodeCmd-request
    (cl:cons ':gcode (gcode msg))
))
;//! \htmlinclude SetGcodeCmd-response.msg.html

(cl:defclass <SetGcodeCmd-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass SetGcodeCmd-response (<SetGcodeCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGcodeCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGcodeCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trajectory_planner-srv:<SetGcodeCmd-response> is deprecated: use trajectory_planner-srv:SetGcodeCmd-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetGcodeCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trajectory_planner-srv:result-val is deprecated.  Use trajectory_planner-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGcodeCmd-response>) ostream)
  "Serializes a message object of type '<SetGcodeCmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGcodeCmd-response>) istream)
  "Deserializes a message object of type '<SetGcodeCmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGcodeCmd-response>)))
  "Returns string type for a service object of type '<SetGcodeCmd-response>"
  "trajectory_planner/SetGcodeCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGcodeCmd-response)))
  "Returns string type for a service object of type 'SetGcodeCmd-response"
  "trajectory_planner/SetGcodeCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGcodeCmd-response>)))
  "Returns md5sum for a message object of type '<SetGcodeCmd-response>"
  "063984922b4c75aaeda3d4d697e63bac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGcodeCmd-response)))
  "Returns md5sum for a message object of type 'SetGcodeCmd-response"
  "063984922b4c75aaeda3d4d697e63bac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGcodeCmd-response>)))
  "Returns full string definition for message of type '<SetGcodeCmd-response>"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGcodeCmd-response)))
  "Returns full string definition for message of type 'SetGcodeCmd-response"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGcodeCmd-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGcodeCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGcodeCmd-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGcodeCmd)))
  'SetGcodeCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGcodeCmd)))
  'SetGcodeCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGcodeCmd)))
  "Returns string type for a service object of type '<SetGcodeCmd>"
  "trajectory_planner/SetGcodeCmd")