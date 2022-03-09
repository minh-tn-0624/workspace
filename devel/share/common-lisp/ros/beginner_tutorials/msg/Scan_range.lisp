; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Scan_range.msg.html

(cl:defclass <Scan_range> (roslisp-msg-protocol:ros-message)
  ((maximum
    :reader maximum
    :initarg :maximum
    :type cl:float
    :initform 0.0)
   (minimum
    :reader minimum
    :initarg :minimum
    :type cl:float
    :initform 0.0))
)

(cl:defclass Scan_range (<Scan_range>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Scan_range>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Scan_range)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Scan_range> is deprecated: use beginner_tutorials-msg:Scan_range instead.")))

(cl:ensure-generic-function 'maximum-val :lambda-list '(m))
(cl:defmethod maximum-val ((m <Scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:maximum-val is deprecated.  Use beginner_tutorials-msg:maximum instead.")
  (maximum m))

(cl:ensure-generic-function 'minimum-val :lambda-list '(m))
(cl:defmethod minimum-val ((m <Scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:minimum-val is deprecated.  Use beginner_tutorials-msg:minimum instead.")
  (minimum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Scan_range>) ostream)
  "Serializes a message object of type '<Scan_range>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maximum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'minimum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Scan_range>) istream)
  "Deserializes a message object of type '<Scan_range>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maximum) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minimum) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Scan_range>)))
  "Returns string type for a message object of type '<Scan_range>"
  "beginner_tutorials/Scan_range")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Scan_range)))
  "Returns string type for a message object of type 'Scan_range"
  "beginner_tutorials/Scan_range")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Scan_range>)))
  "Returns md5sum for a message object of type '<Scan_range>"
  "794426e8c6a60ff752d8c666c7346466")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Scan_range)))
  "Returns md5sum for a message object of type 'Scan_range"
  "794426e8c6a60ff752d8c666c7346466")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Scan_range>)))
  "Returns full string definition for message of type '<Scan_range>"
  (cl:format cl:nil "float64 maximum~%float64 minimum~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Scan_range)))
  "Returns full string definition for message of type 'Scan_range"
  (cl:format cl:nil "float64 maximum~%float64 minimum~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Scan_range>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Scan_range>))
  "Converts a ROS message object to a list"
  (cl:list 'Scan_range
    (cl:cons ':maximum (maximum msg))
    (cl:cons ':minimum (minimum msg))
))
