execute_process(COMMAND "/home/domi/drl_ws/build/hrl_geom/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/domi/drl_ws/build/hrl_geom/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
