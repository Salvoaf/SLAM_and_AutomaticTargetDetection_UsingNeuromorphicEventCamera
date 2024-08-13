
(cl:in-package :asdf)

(defsystem "hello_world-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBoxWithTime" :depends-on ("_package_BoundingBoxWithTime"))
    (:file "_package_BoundingBoxWithTime" :depends-on ("_package"))
    (:file "Event" :depends-on ("_package_Event"))
    (:file "_package_Event" :depends-on ("_package"))
    (:file "EventArray" :depends-on ("_package_EventArray"))
    (:file "_package_EventArray" :depends-on ("_package"))
  ))