
(in-package :asdf)

(defsystem "tricopter-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "TricJoy" :depends-on ("_package"))
    (:file "_package_TricJoy" :depends-on ("_package"))
    ))
