;; Auto-generated. Do not edit!


(when (boundp 'message::text)
  (if (not (find-package "MESSAGE"))
    (make-package "MESSAGE"))
  (shadow 'text (find-package "MESSAGE")))
(unless (find-package "MESSAGE::TEXT")
  (make-package "MESSAGE::TEXT"))
(unless (find-package "MESSAGE::TEXTREQUEST")
  (make-package "MESSAGE::TEXTREQUEST"))
(unless (find-package "MESSAGE::TEXTRESPONSE")
  (make-package "MESSAGE::TEXTRESPONSE"))

(in-package "ROS")





(defclass message::textRequest
  :super ros::object
  :slots (_num1 _num2 _age ))

(defmethod message::textRequest
  (:init
   (&key
    ((:num1 __num1) 0)
    ((:num2 __num2) 0)
    ((:age __age) 0.0)
    )
   (send-super :init)
   (setq _num1 (round __num1))
   (setq _num2 (round __num2))
   (setq _age (float __age))
   self)
  (:num1
   (&optional __num1)
   (if __num1 (setq _num1 __num1)) _num1)
  (:num2
   (&optional __num2)
   (if __num2 (setq _num2 __num2)) _num2)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:serialization-length
   ()
   (+
    ;; int32 _num1
    4
    ;; int32 _num2
    4
    ;; float32 _age
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num1
       (write-long _num1 s)
     ;; int32 _num2
       (write-long _num2 s)
     ;; float32 _age
       (sys::poke _age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num1
     (setq _num1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num2
     (setq _num2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _age
     (setq _age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass message::textResponse
  :super ros::object
  :slots (_num _result ))

(defmethod message::textResponse
  (:init
   (&key
    ((:num __num) 0)
    ((:result __result) 0.0)
    )
   (send-super :init)
   (setq _num (round __num))
   (setq _result (float __result))
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _num
    4
    ;; float32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num
       (write-long _num s)
     ;; float32 _result
       (sys::poke _result (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _result
     (setq _result (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass message::text
  :super ros::object
  :slots ())

(setf (get message::text :md5sum-) "5c43c30d16420052e2be2adf23ac5da0")
(setf (get message::text :datatype-) "message/text")
(setf (get message::text :request) message::textRequest)
(setf (get message::text :response) message::textResponse)

(defmethod message::textRequest
  (:response () (instance message::textResponse :init)))

(setf (get message::textRequest :md5sum-) "5c43c30d16420052e2be2adf23ac5da0")
(setf (get message::textRequest :datatype-) "message/textRequest")
(setf (get message::textRequest :definition-)
      "int32 num1
int32 num2
float32 age
---
int32 num
float32 result
")

(setf (get message::textResponse :md5sum-) "5c43c30d16420052e2be2adf23ac5da0")
(setf (get message::textResponse :datatype-) "message/textResponse")
(setf (get message::textResponse :definition-)
      "int32 num1
int32 num2
float32 age
---
int32 num
float32 result
")



(provide :message/text "5c43c30d16420052e2be2adf23ac5da0")


