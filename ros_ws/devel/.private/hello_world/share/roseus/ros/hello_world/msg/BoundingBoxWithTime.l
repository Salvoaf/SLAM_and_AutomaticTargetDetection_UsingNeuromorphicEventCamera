;; Auto-generated. Do not edit!


(when (boundp 'hello_world::BoundingBoxWithTime)
  (if (not (find-package "HELLO_WORLD"))
    (make-package "HELLO_WORLD"))
  (shadow 'BoundingBoxWithTime (find-package "HELLO_WORLD")))
(unless (find-package "HELLO_WORLD::BOUNDINGBOXWITHTIME")
  (make-package "HELLO_WORLD::BOUNDINGBOXWITHTIME"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxWithTime.msg.html


(defclass hello_world::BoundingBoxWithTime
  :super ros::object
  :slots (_timestamp _x_offset _y_offset _height _width _do_rectify _class_id _class_confidence _track_id ))

(defmethod hello_world::BoundingBoxWithTime
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:x_offset __x_offset) 0)
    ((:y_offset __y_offset) 0)
    ((:height __height) 0)
    ((:width __width) 0)
    ((:do_rectify __do_rectify) nil)
    ((:class_id __class_id) 0)
    ((:class_confidence __class_confidence) 0.0)
    ((:track_id __track_id) 0)
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _x_offset (round __x_offset))
   (setq _y_offset (round __y_offset))
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _do_rectify __do_rectify)
   (setq _class_id (round __class_id))
   (setq _class_confidence (float __class_confidence))
   (setq _track_id (round __track_id))
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:x_offset
   (&optional __x_offset)
   (if __x_offset (setq _x_offset __x_offset)) _x_offset)
  (:y_offset
   (&optional __y_offset)
   (if __y_offset (setq _y_offset __y_offset)) _y_offset)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:do_rectify
   (&optional (__do_rectify :null))
   (if (not (eq __do_rectify :null)) (setq _do_rectify __do_rectify)) _do_rectify)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:class_confidence
   (&optional __class_confidence)
   (if __class_confidence (setq _class_confidence __class_confidence)) _class_confidence)
  (:track_id
   (&optional __track_id)
   (if __track_id (setq _track_id __track_id)) _track_id)
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; int32 _x_offset
    4
    ;; int32 _y_offset
    4
    ;; int32 _height
    4
    ;; int32 _width
    4
    ;; bool _do_rectify
    1
    ;; int32 _class_id
    4
    ;; float32 _class_confidence
    4
    ;; int32 _track_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _x_offset
       (write-long _x_offset s)
     ;; int32 _y_offset
       (write-long _y_offset s)
     ;; int32 _height
       (write-long _height s)
     ;; int32 _width
       (write-long _width s)
     ;; bool _do_rectify
       (if _do_rectify (write-byte -1 s) (write-byte 0 s))
     ;; int32 _class_id
       (write-long _class_id s)
     ;; float32 _class_confidence
       (sys::poke _class_confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _track_id
       (write-long _track_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _x_offset
     (setq _x_offset (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_offset
     (setq _y_offset (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _do_rectify
     (setq _do_rectify (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _class_id
     (setq _class_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _class_confidence
     (setq _class_confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _track_id
     (setq _track_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hello_world::BoundingBoxWithTime :md5sum-) "c5698bfee7fb49c340d5d5281d1a2cca")
(setf (get hello_world::BoundingBoxWithTime :datatype-) "hello_world/BoundingBoxWithTime")
(setf (get hello_world::BoundingBoxWithTime :definition-)
      "float64 timestamp
int32 x_offset
int32 y_offset
int32 height
int32 width
bool do_rectify
int32 class_id
float32 class_confidence
int32 track_id

")



(provide :hello_world/BoundingBoxWithTime "c5698bfee7fb49c340d5d5281d1a2cca")


