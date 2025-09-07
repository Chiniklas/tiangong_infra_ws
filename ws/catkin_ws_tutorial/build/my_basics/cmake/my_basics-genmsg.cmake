# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "my_basics: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imy_basics:/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(my_basics_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg" NAME_WE)
add_custom_target(_my_basics_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_basics" "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(my_basics
  "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_basics
)

### Generating Services

### Generating Module File
_generate_module_cpp(my_basics
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_basics
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(my_basics_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(my_basics_generate_messages my_basics_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg" NAME_WE)
add_dependencies(my_basics_generate_messages_cpp _my_basics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_basics_gencpp)
add_dependencies(my_basics_gencpp my_basics_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_basics_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(my_basics
  "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_basics
)

### Generating Services

### Generating Module File
_generate_module_eus(my_basics
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_basics
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(my_basics_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(my_basics_generate_messages my_basics_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg" NAME_WE)
add_dependencies(my_basics_generate_messages_eus _my_basics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_basics_geneus)
add_dependencies(my_basics_geneus my_basics_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_basics_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(my_basics
  "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_basics
)

### Generating Services

### Generating Module File
_generate_module_lisp(my_basics
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_basics
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(my_basics_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(my_basics_generate_messages my_basics_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg" NAME_WE)
add_dependencies(my_basics_generate_messages_lisp _my_basics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_basics_genlisp)
add_dependencies(my_basics_genlisp my_basics_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_basics_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(my_basics
  "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_basics
)

### Generating Services

### Generating Module File
_generate_module_nodejs(my_basics
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_basics
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(my_basics_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(my_basics_generate_messages my_basics_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg" NAME_WE)
add_dependencies(my_basics_generate_messages_nodejs _my_basics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_basics_gennodejs)
add_dependencies(my_basics_gennodejs my_basics_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_basics_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(my_basics
  "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_basics
)

### Generating Services

### Generating Module File
_generate_module_py(my_basics
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_basics
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(my_basics_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(my_basics_generate_messages my_basics_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/tiangong_infra_ws/ws/catkin_ws_tutorial/src/my_basics/msg/Person.msg" NAME_WE)
add_dependencies(my_basics_generate_messages_py _my_basics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_basics_genpy)
add_dependencies(my_basics_genpy my_basics_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_basics_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_basics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_basics
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(my_basics_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_basics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_basics
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(my_basics_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_basics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_basics
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(my_basics_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_basics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_basics
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(my_basics_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_basics)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_basics\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_basics
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(my_basics_generate_messages_py std_msgs_generate_messages_py)
endif()
