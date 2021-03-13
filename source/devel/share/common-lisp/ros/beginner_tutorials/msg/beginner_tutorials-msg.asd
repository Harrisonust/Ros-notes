
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "my_msg" :depends-on ("_package_my_msg"))
    (:file "_package_my_msg" :depends-on ("_package"))
  ))