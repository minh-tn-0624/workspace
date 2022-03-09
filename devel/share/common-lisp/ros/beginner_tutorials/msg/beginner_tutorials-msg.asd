
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Scan_range" :depends-on ("_package_Scan_range"))
    (:file "_package_Scan_range" :depends-on ("_package"))
  ))