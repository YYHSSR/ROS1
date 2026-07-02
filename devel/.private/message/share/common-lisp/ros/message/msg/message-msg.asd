
(cl:in-package :asdf)

(defsystem "message-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "content" :depends-on ("_package_content"))
    (:file "_package_content" :depends-on ("_package"))
  ))