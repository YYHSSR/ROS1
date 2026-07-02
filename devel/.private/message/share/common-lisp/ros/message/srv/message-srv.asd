
(cl:in-package :asdf)

(defsystem "message-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "text" :depends-on ("_package_text"))
    (:file "_package_text" :depends-on ("_package"))
  ))