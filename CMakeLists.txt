cmake_minimum_required(VERSION 3.0.2)
project(teleop_robot_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
)

set(TELEOP_PROTOBUF_ROOT "$ENV{HOME}/.local/opt/protobuf-3.12.4" CACHE PATH
    "Preferred Protobuf installation for teleop_robot_bridge")
if(EXISTS "${TELEOP_PROTOBUF_ROOT}/bin/protoc")
  set(Protobuf_PROTOC_EXECUTABLE "${TELEOP_PROTOBUF_ROOT}/bin/protoc"
      CACHE FILEPATH "Protocol Buffer compiler" FORCE)
  set(Protobuf_INCLUDE_DIR "${TELEOP_PROTOBUF_ROOT}/include"
      CACHE PATH "Protocol Buffer include directory" FORCE)
  set(Protobuf_LIBRARY "${TELEOP_PROTOBUF_ROOT}/lib/libprotobuf.a"
      CACHE FILEPATH "Protocol Buffer library" FORCE)
endif()
find_package(Protobuf 3.12 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPC REQUIRED grpc++ grpc)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-audio-1.0 gstreamer-base-1.0)
find_package(OpenSSL REQUIRED)

# OpenCV: needed for cv::initUndistortRectifyMap (host-side map gen).
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc)

# yaml-cpp: stereo calibration loader. Optional via pkg-config fallback.
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
  set(YAML_CPP_LIBRARIES_RESOLVED ${YAML_CPP_LIBRARIES})
  set(YAML_CPP_INCLUDE_DIRS_RESOLVED ${YAML_CPP_INCLUDE_DIRS})
else()
  set(YAML_CPP_LIBRARIES_RESOLVED yaml-cpp)
  set(YAML_CPP_INCLUDE_DIRS_RESOLVED "")
endif()

# VPI: lens distortion remap on CUDA backend. Lookup is path-based since
# /opt/nvidia/vpi*/ is the canonical Jetson install location.
find_path(VPI_INCLUDE_DIR vpi/VPI.h
  HINTS /opt/nvidia/vpi3/include /opt/nvidia/vpi2/include /opt/nvidia/vpi1/include
  PATH_SUFFIXES include)
find_library(VPI_LIB nvvpi
  HINTS /opt/nvidia/vpi3/lib/aarch64-linux-gnu /opt/nvidia/vpi2/lib/aarch64-linux-gnu /opt/nvidia/vpi1/lib/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu)
if(VPI_INCLUDE_DIR AND VPI_LIB)
  set(VPI_FOUND TRUE)
  message(STATUS "Found VPI: ${VPI_LIB} (include=${VPI_INCLUDE_DIR})")
else()
  set(VPI_FOUND FALSE)
  message(WARNING "VPI not found; video.undistort feature will not be available")
endif()

# CUDA is optional at configure time so the existing VPI/legacy path can still
# be built on non-Jetson hosts. The Jetson runtime in this workspace provides
# CUDA 11.4 through /usr/local/cuda.
find_program(TRB_CUDA_NVCC nvcc
  HINTS /usr/local/cuda/bin /usr/local/cuda-11.4/bin)
if(TRB_CUDA_NVCC)
  set(CMAKE_CUDA_COMPILER "${TRB_CUDA_NVCC}" CACHE FILEPATH
      "CUDA compiler used by teleop_robot_bridge" FORCE)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(TRB_CUDA_ARCH "87" CACHE STRING
      "CUDA SM architecture for the target Jetson (Orin=87, Xavier=72)")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_${TRB_CUDA_ARCH}")
  find_library(TRB_CUDART_LIBRARY cudart
    HINTS /usr/local/cuda/lib64 /usr/local/cuda-11.4/lib64)
  find_library(TRB_CUDA_DRIVER_LIBRARY cuda
    HINTS /usr/lib/aarch64-linux-gnu /usr/local/cuda/lib64)
  find_library(TRB_EGL_LIBRARY EGL
    HINTS /usr/lib/aarch64-linux-gnu /usr/lib)
  if(TRB_CUDART_LIBRARY AND TRB_CUDA_DRIVER_LIBRARY AND TRB_EGL_LIBRARY)
    set(TRB_CUDA_FOUND TRUE)
    message(STATUS "Found CUDA: ${TRB_CUDA_NVCC} (arch=sm_${TRB_CUDA_ARCH})")
  else()
    set(TRB_CUDA_FOUND FALSE)
    message(WARNING "CUDA runtime/driver/EGL libraries not found; CUDA video backend disabled")
  endif()
else()
  set(TRB_CUDA_FOUND FALSE)
  message(WARNING "nvcc not found; CUDA video backend disabled")
endif()

###########################################################
# Jetson Multimedia API Configuration
###########################################################
set(JETSON_MM_API_ROOT "/usr/src/jetson_multimedia_api" CACHE PATH "Jetson Multimedia API root")
set(JETSON_MM_API_INCLUDE "${JETSON_MM_API_ROOT}/include")
set(JETSON_MM_API_CLASSES_SRC "${JETSON_MM_API_ROOT}/samples/common/classes")
set(TEGRA_LIB_DIR "/usr/lib/aarch64-linux-gnu/tegra")

# Check if Jetson Multimedia API is available
if(EXISTS "${JETSON_MM_API_INCLUDE}/NvVideoEncoder.h")
  set(JETSON_MM_API_FOUND TRUE)
  message(STATUS "Jetson Multimedia API found at ${JETSON_MM_API_ROOT}")
else()
  set(JETSON_MM_API_FOUND FALSE)
  message(WARNING "Jetson Multimedia API not found. Video encoder/decoder support disabled.")
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teleop_proto
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
  DEPENDS Protobuf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${GRPC_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Jetson Multimedia API include directories
if(JETSON_MM_API_FOUND)
  include_directories(
    ${JETSON_MM_API_INCLUDE}
    ${JETSON_MM_API_INCLUDE}/libjpeg-8b
  )
  link_directories(
    ${TEGRA_LIB_DIR}
  )
endif()

set(PROTO_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/protos/signaling.proto
)

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h")
  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc"
           "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h"
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --experimental_allow_proto3_optional
         --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
         -I "${CMAKE_CURRENT_SOURCE_DIR}/protos"
         "${PROTO_FILE}"
    DEPENDS "${PROTO_FILE}"
  )
endforeach()

find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN)
  message(FATAL_ERROR "grpc_cpp_plugin not found. Please install gRPC C++ and ensure grpc_cpp_plugin is in PATH.")
endif()
set(GRPC_CPP_PLUGIN_PROTO3_OPTIONAL
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/grpc_cpp_plugin_proto3_optional.py")

set(GRPC_SRCS)
set(GRPC_HDRS)
foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  list(APPEND GRPC_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.cc")
  list(APPEND GRPC_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.h")
  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.cc" "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.h"
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --experimental_allow_proto3_optional
         --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
         -I "${CMAKE_CURRENT_SOURCE_DIR}/protos"
         --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_PROTO3_OPTIONAL}
         "${PROTO_FILE}"
    DEPENDS "${PROTO_FILE}" "${GRPC_CPP_PLUGIN_PROTO3_OPTIONAL}"
  )
endforeach()

add_library(teleop_proto
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  ${GRPC_SRCS}
  ${GRPC_HDRS}
)

target_link_libraries(teleop_proto
  ${Protobuf_LIBRARIES}
  ${GRPC_LIBRARIES}
)

add_library(teleop_audio STATIC
  src/audio/audio_manager.cpp
  src/audio/audio_module.cpp
)

target_compile_options(teleop_audio PRIVATE ${GSTREAMER_CFLAGS_OTHER})

target_link_libraries(teleop_audio
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

###########################################################
# Jetson Multimedia API Library
###########################################################
if(JETSON_MM_API_FOUND)
  # Build Jetson MM API helper classes as a library
  add_library(jetson_mm_api STATIC
    ${JETSON_MM_API_CLASSES_SRC}/NvBuffer.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvElement.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvElementProfiler.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvJpegDecoder.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvJpegEncoder.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvLogging.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvUtils.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvV4l2Element.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvV4l2ElementPlane.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvVideoDecoder.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvVideoEncoder.cpp
    ${JETSON_MM_API_CLASSES_SRC}/NvBufSurface.cpp
  )

  target_include_directories(jetson_mm_api PUBLIC
    ${JETSON_MM_API_INCLUDE}
    ${JETSON_MM_API_INCLUDE}/libjpeg-8b
  )

  target_link_libraries(jetson_mm_api
    nvbufsurface
    nvbufsurftransform
    nvjpeg
    v4l2
    pthread
  )

  # Video library
  add_library(teleop_video STATIC
    src/video/video_v4l2_capturer.cpp
    src/video/video_decoder.cpp
    src/video/video_converter.cpp
    src/video/video_undistorter.cpp
    src/video/video_encoder.cpp
    src/video/video_stream_manager.cpp
    src/video/video_module.cpp
    src/video/eye_image_publisher.cpp
  )

  if(TRB_CUDA_FOUND)
    target_sources(teleop_video PRIVATE
      src/video/cuda_video_pipeline.cu
    )
  endif()

  target_include_directories(teleop_video PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS_RESOLVED}
  )
  if(VPI_FOUND)
    target_include_directories(teleop_video PRIVATE ${VPI_INCLUDE_DIR})
    target_compile_definitions(teleop_video PRIVATE TRB_VPI_AVAILABLE=1)
  endif()

  target_link_libraries(teleop_video
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES_RESOLVED}
    jetson_mm_api
  )
  if(VPI_FOUND)
    target_link_libraries(teleop_video ${VPI_LIB})
  endif()
  if(TRB_CUDA_FOUND)
    target_link_libraries(teleop_video
      ${TRB_CUDART_LIBRARY}
      ${TRB_CUDA_DRIVER_LIBRARY}
      ${TRB_EGL_LIBRARY}
    )
    target_compile_definitions(teleop_video PRIVATE TRB_CUDA_AVAILABLE=1)
  endif()
  target_compile_definitions(teleop_video PUBLIC JETSON_MM_API_ENABLED=1)
endif()

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/teleop_robot_bridge.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/main_node.cpp
  src/grpc/grpc_manager.cpp
  src/grpc/grpc_module.cpp
  src/udp/control_plane.cpp
  src/udp/fec_v2.cpp
  src/udp/gf256.cpp
  src/udp/pacer.cpp
  src/udp/packet_codec.cpp
  src/udp/send_queue.cpp
  src/udp/udp_manager.cpp
  src/udp/udp_socket.cpp
  src/udp/pose_publisher.cpp
  src/udp/udp_module.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  teleop_proto
  teleop_audio
  OpenSSL::SSL
  OpenSSL::Crypto
)

# Link video library if Jetson MM API is available
if(JETSON_MM_API_FOUND)
  target_link_libraries(${PROJECT_NAME}_node
    teleop_video
  )
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE JETSON_MM_API_ENABLED=1)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_teleop_robot_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
