cmake_minimum_required(VERSION 3.8)
project(teleop_robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig QUIET)
find_package(CUDAToolkit REQUIRED)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES native)
endif()
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
message(STATUS "Found CUDA Toolkit for video CUDA post-processing")

set(TRB_AUDIO_GSTREAMER_ENABLED FALSE)
if(PkgConfig_FOUND)
  pkg_check_modules(GSTREAMER_AUDIO QUIET IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-app-1.0
  )
  if(TARGET PkgConfig::GSTREAMER_AUDIO)
    set(TRB_AUDIO_GSTREAMER_ENABLED TRUE)
    message(STATUS "Found GStreamer audio development packages")
  else()
    message(WARNING "GStreamer audio development packages not found; teleop_robot_bridge audio module will build in stub mode.")
  endif()
else()
  message(WARNING "PkgConfig not found; teleop_robot_bridge audio module will build in stub mode.")
endif()

# gRPC / Protobuf
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

# Helper to find the grpc_cpp_plugin
if(TARGET gRPC::grpc_cpp_plugin)
  set(gRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
else()
  find_program(gRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
  if(NOT gRPC_CPP_PLUGIN_EXECUTABLE)
    message(FATAL_ERROR "grpc_cpp_plugin not found. Ensure gRPC is installed and grpc_cpp_plugin is in PATH or gRPC CMake targets are available.")
  endif()
endif()

# Collect .proto files
file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
if(NOT PROTO_FILES)
  message(WARNING "No .proto files found in ${CMAKE_CURRENT_SOURCE_DIR}/proto")
endif()

# Generate sources
set(PROTO_GENS_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${PROTO_GENS_DIR})

set(PROTO_SRCS "")
set(PROTO_HDRS "")

foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  set(PROTO_SRC "${PROTO_GENS_DIR}/${PROTO_NAME}.pb.cc")
  set(PROTO_HDR "${PROTO_GENS_DIR}/${PROTO_NAME}.pb.h")
  set(GRPC_SRC "${PROTO_GENS_DIR}/${PROTO_NAME}.grpc.pb.cc")
  set(GRPC_HDR "${PROTO_GENS_DIR}/${PROTO_NAME}.grpc.pb.h")

  add_custom_command(
    OUTPUT "${PROTO_SRC}" "${PROTO_HDR}" "${GRPC_SRC}" "${GRPC_HDR}"
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --grpc_out="${PROTO_GENS_DIR}"
      --cpp_out="${PROTO_GENS_DIR}"
      --plugin=protoc-gen-grpc=${gRPC_CPP_PLUGIN_EXECUTABLE}
      -I "${CMAKE_CURRENT_SOURCE_DIR}/proto"
      "${PROTO_FILE}"
    DEPENDS "${PROTO_FILE}"
  )

  list(APPEND PROTO_SRCS ${PROTO_SRC} ${GRPC_SRC})
  list(APPEND PROTO_HDRS ${PROTO_HDR} ${GRPC_HDR})
endforeach()

add_library(${PROJECT_NAME}_grpc_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME}_grpc_proto
  ${Protobuf_LIBRARIES}
  gRPC::grpc++
)
target_include_directories(${PROJECT_NAME}_grpc_proto PUBLIC
  "${PROTO_GENS_DIR}"
)

add_library(video_v4l2_capturer
  src/video/video_v4l2_capturer.cpp
)

target_include_directories(video_v4l2_capturer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(video_v4l2_capturer
  v4l2
  Threads::Threads
)

list(APPEND EXTRA_LIBS video_v4l2_capturer)

add_library(video_converter
  src/video/video_converter.cpp
)

ament_target_dependencies(video_converter
  rclcpp
)

# Jetson Multimedia API (MJPEG hardware decode + NV12 NVMM)
set(JETSON_MM_API_PATH "/usr/src/jetson_multimedia_api")
set(JETSON_COMMON_DIR "${JETSON_MM_API_PATH}/samples/common")
set(JETSON_LIBRARY_DIRS
  /usr/lib/aarch64-linux-gnu/nvidia
  /usr/lib/aarch64-linux-gnu/tegra
)
set(VIDEO_ADVANCED_SOURCES "")
set(VIDEO_ADVANCED_INCLUDE_DIRS "")
set(VIDEO_ADVANCED_LIBS "")

if(EXISTS "${JETSON_MM_API_PATH}")
  message(STATUS "Found Jetson Multimedia API at ${JETSON_MM_API_PATH}")

  find_path(JETSON_JPEG_INCLUDE_DIR
    NAMES jpeglib.h
    PATHS
      "${JETSON_MM_API_PATH}/include/libjpeg-9f"
      "${JETSON_MM_API_PATH}/include/libjpeg-8b"
    NO_DEFAULT_PATH
  )
  if(NOT JETSON_JPEG_INCLUDE_DIR)
    message(FATAL_ERROR
      "Jetson Multimedia API JPEG headers not found. Expected jpeglib.h under "
      "${JETSON_MM_API_PATH}/include/libjpeg-9f or ${JETSON_MM_API_PATH}/include/libjpeg-8b")
  endif()
  message(STATUS "Using Jetson JPEG headers: ${JETSON_JPEG_INCLUDE_DIR}")

  target_include_directories(video_v4l2_capturer PUBLIC
    ${JETSON_MM_API_PATH}/include
  )

  # Jetson Multimedia API sources are third-party; keep our code strict, but
  # silence known noisy warnings from upstream headers/samples.
  target_compile_options(video_converter PRIVATE
    -Wno-pedantic
    -Wno-missing-field-initializers
    -Wno-unused-parameter
  )

  target_compile_definitions(video_converter PUBLIC TRB_HAS_JETSON_MM_API=1)
  target_sources(video_converter PRIVATE
    src/video/cuda_yuv422_converter.cu
  )
  set_source_files_properties(src/video/cuda_yuv422_converter.cu PROPERTIES LANGUAGE CUDA)
  target_include_directories(video_converter PUBLIC ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(video_converter CUDA::cudart CUDA::cuda_driver)

  target_include_directories(video_converter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${JETSON_JPEG_INCLUDE_DIR}
    ${JETSON_MM_API_PATH}/include
    ${JETSON_COMMON_DIR}/classes
  )

  find_library(JETSON_NVV4L2_LIBRARY
    NAMES nvv4l2
    PATHS ${JETSON_LIBRARY_DIRS}
  )
  find_library(JETSON_NVBUFSURFACE_LIBRARY
    NAMES nvbufsurface
    PATHS ${JETSON_LIBRARY_DIRS}
  )
  find_library(JETSON_NVBUFSURFTRANSFORM_LIBRARY
    NAMES nvbufsurftransform
    PATHS ${JETSON_LIBRARY_DIRS}
  )
  find_library(JETSON_NVMM_JPEG_LIBRARY
    NAMES nvmm_jpeg
    PATHS ${JETSON_LIBRARY_DIRS}
  )

  if(NOT JETSON_NVV4L2_LIBRARY OR
     NOT JETSON_NVBUFSURFACE_LIBRARY OR
     NOT JETSON_NVBUFSURFTRANSFORM_LIBRARY OR
     NOT JETSON_NVMM_JPEG_LIBRARY)
    message(FATAL_ERROR
      "Jetson multimedia libraries not found in configured search paths. "
      "Expected nvv4l2, nvbufsurface, nvmm_jpeg under: ${JETSON_LIBRARY_DIRS}")
  endif()

  target_link_libraries(video_v4l2_capturer
    ${JETSON_NVBUFSURFACE_LIBRARY}
  )

  target_sources(video_converter PRIVATE
    "${JETSON_COMMON_DIR}/classes/NvJpegDecoder.cpp"
    src/video/nvbufsurface_shim.cpp
    "${JETSON_COMMON_DIR}/classes/NvBuffer.cpp"
    "${JETSON_COMMON_DIR}/classes/NvElement.cpp"
    "${JETSON_COMMON_DIR}/classes/NvLogging.cpp"
    "${JETSON_COMMON_DIR}/classes/NvElementProfiler.cpp"
    "${JETSON_COMMON_DIR}/classes/NvUtils.cpp"
    "${JETSON_COMMON_DIR}/classes/NvV4l2Element.cpp"
    "${JETSON_COMMON_DIR}/classes/NvV4l2ElementPlane.cpp"
  )

  target_link_libraries(video_converter
    Threads::Threads
    v4l2
    ${JETSON_NVV4L2_LIBRARY}
    ${JETSON_NVBUFSURFACE_LIBRARY}
    ${JETSON_NVBUFSURFTRANSFORM_LIBRARY}
    ${JETSON_NVMM_JPEG_LIBRARY}
  )

  # Video Encoder (H.264 hardware encode)
  add_library(video_encoder
    src/video/video_encoder.cpp
  )

  # video_encoder now uses TeleopLogger, which depends on rclcpp.
  ament_target_dependencies(video_encoder
    rclcpp
  )

  target_compile_options(video_encoder PRIVATE
    -Wno-pedantic
    -Wno-missing-field-initializers
    -Wno-unused-parameter
  )

  target_compile_definitions(video_encoder PUBLIC TRB_HAS_JETSON_MM_API=1)

  target_include_directories(video_encoder PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CUDAToolkit_INCLUDE_DIRS}
    ${JETSON_JPEG_INCLUDE_DIR}
    ${JETSON_MM_API_PATH}/include
    ${JETSON_COMMON_DIR}/classes
  )

  target_sources(video_encoder PRIVATE
    "${JETSON_COMMON_DIR}/classes/NvVideoEncoder.cpp"
    "${JETSON_COMMON_DIR}/classes/NvBuffer.cpp"
    "${JETSON_COMMON_DIR}/classes/NvElement.cpp"
    "${JETSON_COMMON_DIR}/classes/NvLogging.cpp"
    "${JETSON_COMMON_DIR}/classes/NvElementProfiler.cpp"
    "${JETSON_COMMON_DIR}/classes/NvUtils.cpp"
    "${JETSON_COMMON_DIR}/classes/NvV4l2Element.cpp"
    "${JETSON_COMMON_DIR}/classes/NvV4l2ElementPlane.cpp"
  )

  target_link_libraries(video_encoder
    Threads::Threads
    v4l2
    ${JETSON_NVV4L2_LIBRARY}
    ${JETSON_NVBUFSURFACE_LIBRARY}
  )

  list(APPEND VIDEO_ADVANCED_SOURCES
    src/video/video_decoder.cpp
    src/video/video_undistorter.cpp
    src/video/video_scaler.cpp
    src/video/eye_image_publisher.cpp
    src/video/wrist_camera_publisher.cpp
    src/video/cuda_eye_image_processor.cu
    src/video/cuda_nv12_undistorter.cu
  )
  list(APPEND VIDEO_ADVANCED_INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
    ${JETSON_MM_API_PATH}/include
  )
  list(APPEND VIDEO_ADVANCED_LIBS
    ${OpenCV_LIBS}
    yaml-cpp
    ${JETSON_NVBUFSURFACE_LIBRARY}
    ${JETSON_NVBUFSURFTRANSFORM_LIBRARY}
    ${JETSON_NVV4L2_LIBRARY}
    CUDA::cudart
    CUDA::cuda_driver
  )
  set_source_files_properties(
    src/video/video_stream_manager.cpp
    src/video/video_decoder.cpp
    PROPERTIES COMPILE_OPTIONS "-Wno-pedantic"
  )
  set_source_files_properties(
    src/video/cuda_eye_image_processor.cu
    src/video/cuda_nv12_undistorter.cu
    PROPERTIES
      LANGUAGE CUDA
      COMPILE_OPTIONS "-Wno-pedantic"
  )
else()
  message(FATAL_ERROR "Jetson Multimedia API not found at ${JETSON_MM_API_PATH}. CUDA video pipeline cannot be built.")
endif()



list(APPEND EXTRA_LIBS video_converter video_encoder)

file(GLOB TELEOP_MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
list(SORT TELEOP_MSG_FILES)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${TELEOP_MSG_FILES}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_executable(${PROJECT_NAME}_node
  src/main_node.cpp
  src/audio/audio_manager.cpp
  src/audio/audio_module.cpp
  src/audio/audio_receiver.cpp
  src/grpc/grpc_manager.cpp
  src/grpc/grpc_module.cpp
  src/utils/teleop_logger.cpp
  src/video/video_recorder.cpp
  src/video/video_module.cpp
  src/video/video_stream_manager.cpp
  ${VIDEO_ADVANCED_SOURCES}
  src/udp/udp_socket.cpp
  src/udp/gf256.cpp
  src/udp/packet_codec.cpp
  src/udp/control_plane.cpp
  src/udp/send_queue.cpp
  src/udp/pacer.cpp
  src/udp/fec_v2.cpp
  src/udp/udp_manager.cpp
  src/udp/udp_module.cpp
  src/udp/pose_udp_receiver.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  "${PROTO_GENS_DIR}"
  ${VIDEO_ADVANCED_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

if(TRB_AUDIO_GSTREAMER_ENABLED)
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE TRB_HAS_GSTREAMER_AUDIO=1)
else()
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE TRB_HAS_GSTREAMER_AUDIO=0)
endif()

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(TRB_AUDIO_GSTREAMER_ENABLED)
  target_link_libraries(${PROJECT_NAME}_node PkgConfig::GSTREAMER_AUDIO)
endif()

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_grpc_proto
  v4l2
  Threads::Threads
  OpenSSL::SSL
  OpenSSL::Crypto
  ${cpp_typesupport_target}
  ${EXTRA_LIBS}
  ${VIDEO_ADVANCED_LIBS}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Scoped header install: install under include/${PROJECT_NAME}
install(FILES
  include/main_node.hpp
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/video
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/grpc
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/audio
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/udp
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/telemetry
  DESTINATION include/${PROJECT_NAME}
)


install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
