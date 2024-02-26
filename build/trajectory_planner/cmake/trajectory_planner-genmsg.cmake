# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "trajectory_planner: 0 messages, 5 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(trajectory_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv" NAME_WE)
add_custom_target(_trajectory_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trajectory_planner" "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv" ""
)

get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv" NAME_WE)
add_custom_target(_trajectory_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trajectory_planner" "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv" ""
)

get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv" NAME_WE)
add_custom_target(_trajectory_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trajectory_planner" "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv" ""
)

get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv" NAME_WE)
add_custom_target(_trajectory_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trajectory_planner" "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv" ""
)

get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv" NAME_WE)
add_custom_target(_trajectory_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trajectory_planner" "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_cpp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_cpp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_cpp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_cpp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner
)

### Generating Module File
_generate_module_cpp(trajectory_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(trajectory_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(trajectory_planner_generate_messages trajectory_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_cpp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_cpp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_cpp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_cpp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_cpp _trajectory_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_planner_gencpp)
add_dependencies(trajectory_planner_gencpp trajectory_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner
)
_generate_srv_eus(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner
)
_generate_srv_eus(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner
)
_generate_srv_eus(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner
)
_generate_srv_eus(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner
)

### Generating Module File
_generate_module_eus(trajectory_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(trajectory_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(trajectory_planner_generate_messages trajectory_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_eus _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_eus _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_eus _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_eus _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_eus _trajectory_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_planner_geneus)
add_dependencies(trajectory_planner_geneus trajectory_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_lisp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_lisp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_lisp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner
)
_generate_srv_lisp(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner
)

### Generating Module File
_generate_module_lisp(trajectory_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(trajectory_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(trajectory_planner_generate_messages trajectory_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_lisp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_lisp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_lisp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_lisp _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_lisp _trajectory_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_planner_genlisp)
add_dependencies(trajectory_planner_genlisp trajectory_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner
)
_generate_srv_nodejs(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner
)
_generate_srv_nodejs(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner
)
_generate_srv_nodejs(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner
)
_generate_srv_nodejs(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner
)

### Generating Module File
_generate_module_nodejs(trajectory_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(trajectory_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(trajectory_planner_generate_messages trajectory_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_nodejs _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_nodejs _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_nodejs _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_nodejs _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_nodejs _trajectory_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_planner_gennodejs)
add_dependencies(trajectory_planner_gennodejs trajectory_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner
)
_generate_srv_py(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner
)
_generate_srv_py(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner
)
_generate_srv_py(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner
)
_generate_srv_py(trajectory_planner
  "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner
)

### Generating Module File
_generate_module_py(trajectory_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(trajectory_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(trajectory_planner_generate_messages trajectory_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetJointCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_py _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetHomeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_py _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/GetPoseCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_py _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetCartCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_py _trajectory_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/domi/drl_ws/src/trajectory_planner/srv/SetGcodeCmd.srv" NAME_WE)
add_dependencies(trajectory_planner_generate_messages_py _trajectory_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_planner_genpy)
add_dependencies(trajectory_planner_genpy trajectory_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(trajectory_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/trajectory_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(trajectory_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(trajectory_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/trajectory_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(trajectory_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(trajectory_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
