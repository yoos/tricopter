; Auto-generated. Do not edit!


(in-package tricopter-msg)


;//! \htmlinclude TricJoy.msg.html

(defclass <TricJoy> (ros-message)
  ((axisInputs
    :reader axisInputs-val
    :initarg :axisInputs
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (buttonInputs
    :reader buttonInputs-val
    :initarg :buttonInputs
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0)))
)
(defmethod serialize ((msg <TricJoy>) ostream)
  "Serializes a message object of type '<TricJoy>"
  (let ((__ros_arr_len (length (slot-value msg 'axisInputs))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'axisInputs))
  (let ((__ros_arr_len (length (slot-value msg 'buttonInputs))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'buttonInputs))
)
(defmethod deserialize ((msg <TricJoy>) istream)
  "Deserializes a message object of type '<TricJoy>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'axisInputs) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'axisInputs)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'buttonInputs) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'buttonInputs)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<TricJoy>)))
  "Returns string type for a message object of type '<TricJoy>"
  "tricopter/TricJoy")
(defmethod md5sum ((type (eql '<TricJoy>)))
  "Returns md5sum for a message object of type '<TricJoy>"
  "e59d678015c833dc1ec1269225cfa7de")
(defmethod message-definition ((type (eql '<TricJoy>)))
  "Returns full string definition for message of type '<TricJoy>"
  (format nil "int32[] axisInputs~%int32[] buttonInputs~%~%~%"))
(defmethod serialization-length ((msg <TricJoy>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'axisInputs) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'buttonInputs) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <TricJoy>))
  "Converts a ROS message object to a list"
  (list '<TricJoy>
    (cons ':axisInputs (axisInputs-val msg))
    (cons ':buttonInputs (buttonInputs-val msg))
))
