; Auto-generated. Do not edit!


(cl:in-package message-srv)


;//! \htmlinclude text-request.msg.html

(cl:defclass <text-request> (roslisp-msg-protocol:ros-message)
  ((num1
    :reader num1
    :initarg :num1
    :type cl:integer
    :initform 0)
   (num2
    :reader num2
    :initarg :num2
    :type cl:integer
    :initform 0)
   (age
    :reader age
    :initarg :age
    :type cl:float
    :initform 0.0))
)

(cl:defclass text-request (<text-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <text-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'text-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message-srv:<text-request> is deprecated: use message-srv:text-request instead.")))

(cl:ensure-generic-function 'num1-val :lambda-list '(m))
(cl:defmethod num1-val ((m <text-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-srv:num1-val is deprecated.  Use message-srv:num1 instead.")
  (num1 m))

(cl:ensure-generic-function 'num2-val :lambda-list '(m))
(cl:defmethod num2-val ((m <text-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-srv:num2-val is deprecated.  Use message-srv:num2 instead.")
  (num2 m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <text-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-srv:age-val is deprecated.  Use message-srv:age instead.")
  (age m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <text-request>) ostream)
  "Serializes a message object of type '<text-request>"
  (cl:let* ((signed (cl:slot-value msg 'num1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'age))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <text-request>) istream)
  "Deserializes a message object of type '<text-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'age) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<text-request>)))
  "Returns string type for a service object of type '<text-request>"
  "message/textRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'text-request)))
  "Returns string type for a service object of type 'text-request"
  "message/textRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<text-request>)))
  "Returns md5sum for a message object of type '<text-request>"
  "5c43c30d16420052e2be2adf23ac5da0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'text-request)))
  "Returns md5sum for a message object of type 'text-request"
  "5c43c30d16420052e2be2adf23ac5da0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<text-request>)))
  "Returns full string definition for message of type '<text-request>"
  (cl:format cl:nil "int32 num1~%int32 num2~%float32 age~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'text-request)))
  "Returns full string definition for message of type 'text-request"
  (cl:format cl:nil "int32 num1~%int32 num2~%float32 age~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <text-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <text-request>))
  "Converts a ROS message object to a list"
  (cl:list 'text-request
    (cl:cons ':num1 (num1 msg))
    (cl:cons ':num2 (num2 msg))
    (cl:cons ':age (age msg))
))
;//! \htmlinclude text-response.msg.html

(cl:defclass <text-response> (roslisp-msg-protocol:ros-message)
  ((num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0)
   (result
    :reader result
    :initarg :result
    :type cl:float
    :initform 0.0))
)

(cl:defclass text-response (<text-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <text-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'text-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message-srv:<text-response> is deprecated: use message-srv:text-response instead.")))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <text-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-srv:num-val is deprecated.  Use message-srv:num instead.")
  (num m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <text-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-srv:result-val is deprecated.  Use message-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <text-response>) ostream)
  "Serializes a message object of type '<text-response>"
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <text-response>) istream)
  "Deserializes a message object of type '<text-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'result) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<text-response>)))
  "Returns string type for a service object of type '<text-response>"
  "message/textResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'text-response)))
  "Returns string type for a service object of type 'text-response"
  "message/textResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<text-response>)))
  "Returns md5sum for a message object of type '<text-response>"
  "5c43c30d16420052e2be2adf23ac5da0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'text-response)))
  "Returns md5sum for a message object of type 'text-response"
  "5c43c30d16420052e2be2adf23ac5da0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<text-response>)))
  "Returns full string definition for message of type '<text-response>"
  (cl:format cl:nil "int32 num~%float32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'text-response)))
  "Returns full string definition for message of type 'text-response"
  (cl:format cl:nil "int32 num~%float32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <text-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <text-response>))
  "Converts a ROS message object to a list"
  (cl:list 'text-response
    (cl:cons ':num (num msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'text)))
  'text-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'text)))
  'text-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'text)))
  "Returns string type for a service object of type '<text>"
  "message/text")