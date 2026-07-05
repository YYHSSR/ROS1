execute_process(COMMAND "/home/zf/catkin_ws/build/fkie_master_discovery/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zf/catkin_ws/build/fkie_master_discovery/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
