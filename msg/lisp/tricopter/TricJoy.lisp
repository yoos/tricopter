; Auto-generated. Do not edit!


(in-package tricopter-msg)


;//! \htmlinclude TricJoy.msg.html

(defclass <TricJoy> (ros-message)
  ((axes
    :reader axes-val
    :initarg :axes
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0)))
)
(defmethod serialize ((msg <TricJoy>) ostream)
  "Serializes a message object of type '<TricJoy>"
  (let ((__ros_arr_len (length (slot-value msg 'axes))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'axes))
)
(defmethod deserialize ((msg <TricJoy>) istream)
  "Deserializes a message object of type '<TricJoy>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'axes) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'axes)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<TricJoy>)))
  "Returns string type for a message object of type '<TricJoy>"
  "tricopter/TricJoy")
(defmethod md5sum ((type (eql '<TricJoy>)))
  "Returns md5sum for a message object of type '<TricJoy>"
  "8ae8b3882249893c144fe391d11d46f1")
(defmethod message-definition ((type (eql '<TricJoy>)))
  "Returns full string definition for message of type '<TricJoy>"
  (format nil "byte[] axes~%~%~%"))
(defmethod serialization-length ((msg <TricJoy>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'axes) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
))
(defmethod ros-message-to-list ((msg <TricJoy>))
  "Converts a ROS message object to a list"
  (list '<TricJoy>
    (cons ':axes (axes-val msg))
))
