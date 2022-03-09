;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::scan_range)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'scan_range (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::SCAN_RANGE")
  (make-package "BEGINNER_TUTORIALS::SCAN_RANGE"))

(in-package "ROS")
;;//! \htmlinclude scan_range.msg.html


(defclass beginner_tutorials::scan_range
  :super ros::object
  :slots (_min _max ))

(defmethod beginner_tutorials::scan_range
  (:init
   (&key
    ((:min __min) 0)
    ((:max __max) 0)
    )
   (send-super :init)
   (setq _min (round __min))
   (setq _max (round __max))
   self)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:max
   (&optional __max)
   (if __max (setq _max __max)) _max)
  (:serialization-length
   ()
   (+
    ;; int64 _min
    8
    ;; int64 _max
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _min
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _min (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _min) (= (length (_min . bv)) 2)) ;; bignum
              (write-long (ash (elt (_min . bv) 0) 0) s)
              (write-long (ash (elt (_min . bv) 1) -1) s))
             ((and (class _min) (= (length (_min . bv)) 1)) ;; big1
              (write-long (elt (_min . bv) 0) s)
              (write-long (if (>= _min 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _min s)(write-long (if (>= _min 0) 0 #xffffffff) s)))
     ;; int64 _max
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _max (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _max) (= (length (_max . bv)) 2)) ;; bignum
              (write-long (ash (elt (_max . bv) 0) 0) s)
              (write-long (ash (elt (_max . bv) 1) -1) s))
             ((and (class _max) (= (length (_max . bv)) 1)) ;; big1
              (write-long (elt (_max . bv) 0) s)
              (write-long (if (>= _max 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _max s)(write-long (if (>= _max 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _min
#+(or :alpha :irix6 :x86_64)
      (setf _min (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _min (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _max
#+(or :alpha :irix6 :x86_64)
      (setf _max (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _max (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get beginner_tutorials::scan_range :md5sum-) "a1d10007dc73ba7537d4cef82691d404")
(setf (get beginner_tutorials::scan_range :datatype-) "beginner_tutorials/scan_range")
(setf (get beginner_tutorials::scan_range :definition-)
      "int64 min
int64 max

")



(provide :beginner_tutorials/scan_range "a1d10007dc73ba7537d4cef82691d404")


