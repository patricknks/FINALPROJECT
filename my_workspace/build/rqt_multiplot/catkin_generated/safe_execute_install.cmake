execute_process(COMMAND "/home/patricknks/FINALPROJECT/my_workspace/build/rqt_multiplot/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/patricknks/FINALPROJECT/my_workspace/build/rqt_multiplot/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
