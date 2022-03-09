;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::Scan_range)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'Scan_range (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::SCAN_RANGE")
  (make-package "BEGINNER_TUTORIALS::SCAN_RANGE"))

(in-package "ROS")
;;//! \htmlinclude Scan_range.msg.html


(defclass beginner_tutorials::Scan_range
  :super ros::object
  :slots (_maximum _minimum ))

(defmethod beginner_tutorials::Scan_range
  (:init
   (&key
    ((:maximum __maximum) 0.0)
    ((:minimum __minimum) 0.0)
    )
   (send-super :init)
   (setq _maximum (float __maximum))
   (setq _minimum (float __minimum))
   self)
  (:maximum
   (&optional __maximum)
   (if __maximum (setq _maximum __maximum)) _maximum)
  (:minimum
   (&optional __minimum)
   (if __minimum (setq _minimum __minimum)) _minimum)
  (:serialization-length
   ()
   (+
    ;; float64 _maximum
    8
    ;; float64 _minimum
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _maximum
       (sys::poke _maximum (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _minimum
       (sys::poke _minimum (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _maximum
     (setq _maximum (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _minimum
     (setq _minimum (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get beginner_tutorials::Scan_range :md5sum-) "794426e8c6a60ff752d8c666c7346466")
(setf (get beginner_tutorials::Scan_range :datatype-) "beginner_tutorials/Scan_range")
(setf (get beginner_tutorials::Scan_range :definition-)
      "float64 maximum
float64 minimum
")



(provide :beginner_tutorials/Scan_range "794426e8c6a60ff752d8c666c7346466")


