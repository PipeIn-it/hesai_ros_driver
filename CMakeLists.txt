cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hesai_ros_driver)

#=======================================
# Version information
#=======================================
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  7)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
message(=============================================================)
message("-- ROS_VERSION is $ENV{ROS_VERSION}")
message(=============================================================)


#========================
# Project details / setup
#========================
set(PROJECT_NAME hesai_ros_driver)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")  # the ros2 humble requires c++17
add_definitions(-std=c++17)
else()
add_definitions(-std=c++14)
endif()

add_compile_options(-Wall)

# test_xt_block_timing drives the SDK's header-only Udp6_1 parser directly.
set(HESAI_XT_TEST_INCLUDES
  src/driver/HesaiLidar_SDK_2.0/libhesai
  src/driver/HesaiLidar_SDK_2.0/libhesai/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/Lidar
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/src
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpProtocol)

#========================
# Dependencies Setup
#========================

#ROS#
#Catkin#
if($ENV{ROS_VERSION} MATCHES "1")
  find_package(roscpp 1.12 QUIET)
  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})
  add_definitions(-DROS_FOUND)
  add_definitions(-DRUN_IN_ROS_WORKSPACE)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    diagnostic_msgs
    message_generation
    roslib
    nodelet
    pluginlib)
  include_directories(${catkin_INCLUDE_DIRS})

  add_message_files(
    FILES
    "UdpPacket.msg"
    "UdpFrame.msg"
    "LossPacket.msg"
    "Ptp.msg"
    "Firetime.msg"
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )


  catkin_package(CATKIN_DEPENDS
    sensor_msgs
    diagnostic_msgs
    roslib
    nodelet
    pluginlib)

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_time_sync_policy test/test_time_sync_policy.cpp)
    target_include_directories(test_time_sync_policy PRIVATE src/manager)
    # The test has no main(); ament_add_gtest links gtest_main by default.
    target_link_libraries(test_time_sync_policy gtest_main)
    catkin_add_gtest(test_xt_block_timing test/test_xt_block_timing.cpp)
    target_include_directories(test_xt_block_timing PRIVATE ${HESAI_XT_TEST_INCLUDES})
    target_link_libraries(test_xt_block_timing gtest_main platutils_lib)
  endif()

endif($ENV{ROS_VERSION} MATCHES "1")

#ROS2#
if($ENV{ROS_VERSION} MATCHES "2")

  find_package(rclcpp QUIET)
  if(rclcpp_FOUND)

    message(=============================================================)
    message("-- ROS2 Found. ROS2 Support is turned On.")
    message(=============================================================)

    add_definitions(-DROS2_FOUND)
    include_directories(${rclcpp_INCLUDE_DIRS})
    set(CMAKE_CXX_STANDARD 14)

    find_package(ament_cmake REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(diagnostic_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rclcpp_action REQUIRED)
    find_package(rosidl_typesupport_c REQUIRED) 
    find_package(rosidl_default_generators REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(std_msgs REQUIRED)
    
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/msg_ros2/UdpPacket.msg"
      "msg/msg_ros2/UdpFrame.msg"
      "msg/msg_ros2/Firetime.msg"
      "msg/msg_ros2/Ptp.msg"
      "msg/msg_ros2/LossPacket.msg"
      DEPENDENCIES builtin_interfaces std_msgs
    )
    ament_export_dependencies(rosidl_default_runtime)                 
    
  else(rclcpp_FOUND)

    message(=============================================================)
    message("-- ROS2 Not Found. ROS2 Support is turned Off.")
    message(=============================================================)

  endif(rclcpp_FOUND )
endif($ENV{ROS_VERSION} MATCHES "2")


#Others#
find_package(yaml-cpp REQUIRED)

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(src/driver/HesaiLidar_SDK_2.0)

#========================
# Build Setup
#========================

# add_executable(hesai_ros_driver_node
#                node/hesai_ros_driver_node.cpp
#                src/manager/node_manager.cpp
#                )
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# if install different cuda version, set the cuda path, like cuda-11.4
# set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4/)
# find_package(CUDA REQUIRED)
find_package(CUDA )

if(CUDA_FOUND)
  
  message(=============================================================)
  message("-- CUDA Found. CUDA Support is turned On.")
  message(=============================================================)

  link_directories($ENV{CUDA_PATH}/lib/x64)
  set(CUDA_NVCC_FLAGS "-arch=sm_75;-O2")#根据具体GPU性能更改算力参数
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -rdc=true")
  list(APPEND CUDA_NVCC_FLAGS -Xcompiler -fPIC)

  CUDA_ADD_EXECUTABLE(hesai_ros_driver_node
                node/hesai_ros_driver_node.cu
                src/manager/node_manager.cu
                ./src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/src/buffer.cu
                )
  set(CUDA_LIBS "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so")

  target_link_libraries(hesai_ros_driver_node                   
                ${YAML_CPP_LIBRARIES}
                ${Boost_LIBRARIES}
                source_lib
                container_lib
                ptcClient_lib
                ptcParser_lib
                log_lib
                ${CUDA_LIBS}
                # libhesai
    )
else(CUDA_FOUND)

  message(=============================================================)
  message("-- CUDA Not Found. CUDA Support is turned Off.")
  message(=============================================================)
  add_executable(hesai_ros_driver_node
                node/hesai_ros_driver_node.cc
                src/manager/node_manager.cc
                )
  target_link_libraries(hesai_ros_driver_node                   
              ${YAML_CPP_LIBRARIES}
              ${Boost_LIBRARIES}
              source_lib
              container_lib
              ptcClient_lib
              ptcParser_lib
              log_lib
              # libhesai
  )            

endif(CUDA_FOUND)


# Shared SDK include directories (used by node and component/nodelet targets)
set(HESAI_SDK_INCLUDE_DIRS
  src/driver/HesaiLidar_SDK_2.0/
  src/driver/HesaiLidar_SDK_2.0/libhesai
  src/driver/HesaiLidar_SDK_2.0/libhesai/Lidar
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/src
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpProtocol
  src/driver/HesaiLidar_SDK_2.0/libhesai/Source/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/Container/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/Container/src
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/src
  src/driver/HesaiLidar_SDK_2.0/libhesai/PtcClient/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/PtcParser/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/PtcParser
  src/driver/HesaiLidar_SDK_2.0/libhesai/Logger/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/include
  src/driver/HesaiLidar_SDK_2.0/driver
  src/manager
  src/msg/ros_msg
  src/msg/rs_msg
  src/utility
  ${CMAKE_CURRENT_BINARY_DIR}
)

# Shared SDK link libraries
set(HESAI_SDK_LINK_LIBS
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  source_lib
  container_lib
  ptcClient_lib
  ptcParser_lib
  log_lib
)

  target_include_directories(hesai_ros_driver_node PRIVATE ${HESAI_SDK_INCLUDE_DIRS})  

#Ros#
if($ENV{ROS_VERSION} MATCHES "1")
  target_link_libraries(hesai_ros_driver_node  ${ROS_LIBS})
  install(TARGETS hesai_ros_driver_node
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  # Hesai Driver Nodelet (for composed pipelines)
  add_library(hesai_driver_nodelet SHARED
    src/manager/hesai_driver_nodelet.cpp
  )
  target_include_directories(hesai_driver_nodelet PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
  target_link_libraries(hesai_driver_nodelet
    ${HESAI_SDK_LINK_LIBS}
    ${ROS_LIBS}
    ${catkin_LIBRARIES}
  )
  if(CUDA_FOUND)
    target_link_libraries(hesai_driver_nodelet ${CUDA_LIBS})
  endif()
  add_dependencies(hesai_driver_nodelet
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  install(TARGETS hesai_driver_nodelet
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  # The driver's own tests (9B), here where the SDK targets and include lists exist.
  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_driver_accounting test/test_driver_accounting.cpp)
    target_include_directories(test_driver_accounting PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
    target_link_libraries(test_driver_accounting gtest_main)
    catkin_add_gtest(test_config_defaults test/test_config_defaults.cpp)
    target_include_directories(test_config_defaults PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
    target_link_libraries(test_config_defaults ${HESAI_SDK_LINK_LIBS} ${ROS_LIBS}
      ${catkin_LIBRARIES} gtest_main)
    catkin_add_gtest(test_ros1_pointcloud_layout test/test_ros1_pointcloud_layout.cpp)
    target_include_directories(test_ros1_pointcloud_layout PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
    target_link_libraries(test_ros1_pointcloud_layout ${HESAI_SDK_LINK_LIBS} ${ROS_LIBS}
      ${catkin_LIBRARIES} gtest_main)
    add_dependencies(test_ros1_pointcloud_layout
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # The SDK's loss counters (9B.2, H3), through the XT32's header-only parser.
    catkin_add_gtest(test_sdk_counters test/test_sdk_counters.cpp)
    target_include_directories(test_sdk_counters PRIVATE ${HESAI_XT_TEST_INCLUDES})
    target_link_libraries(test_sdk_counters gtest_main platutils_lib)
    # The CUDA node's frame-loop step, as host code (review of 9B, finding 3): no GPU needed.
    catkin_add_gtest(test_gpu_frame_step test/test_gpu_frame_step.cpp)
    target_include_directories(test_gpu_frame_step PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
    target_link_libraries(test_gpu_frame_step gtest_main)
    # The driver's own 9B call sites through InitImpl (review of 9B, finding 6): needs a master.
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_ros1_source_driver test/source_driver_ros1.test
      test/test_ros1_source_driver.cpp)
    if(TARGET test_ros1_source_driver)
      target_include_directories(test_ros1_source_driver PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
      target_link_libraries(test_ros1_source_driver ${HESAI_SDK_LINK_LIBS} ${ROS_LIBS}
        ${catkin_LIBRARIES})
      add_dependencies(test_ros1_source_driver
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
  endif()
endif($ENV{ROS_VERSION} MATCHES "1")


#Ros2#

if($ENV{ROS_VERSION} MATCHES "2")
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp_components REQUIRED)

  ament_target_dependencies(hesai_ros_driver_node
  "ament_index_cpp"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "diagnostic_msgs"
  # "tf2_geometry_msgs"
  )
  rosidl_target_interfaces(hesai_ros_driver_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")

  # Hesai Driver Component (for composed pipelines with zero-copy IPC)
  add_library(hesai_driver_component SHARED
    src/manager/hesai_driver_component.cpp
  )
  target_include_directories(hesai_driver_component PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
  target_link_libraries(hesai_driver_component ${HESAI_SDK_LINK_LIBS})
  if(CUDA_FOUND)
    target_link_libraries(hesai_driver_component ${CUDA_LIBS})
  endif()
  ament_target_dependencies(hesai_driver_component
    rclcpp rclcpp_components sensor_msgs std_msgs diagnostic_msgs ament_index_cpp
  )
  rosidl_target_interfaces(hesai_driver_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rclcpp_components_register_nodes(hesai_driver_component
    "hesai_ros_driver::HesaiDriverComponent"
  )

  install(TARGETS
    hesai_ros_driver_node
    DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
    hesai_driver_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    rviz
    config
    DESTINATION share/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_time_sync_policy test/test_time_sync_policy.cpp)
    target_include_directories(test_time_sync_policy PRIVATE src/manager)
    ament_add_gtest(test_xt_block_timing test/test_xt_block_timing.cpp)
    target_include_directories(test_xt_block_timing PRIVATE ${HESAI_XT_TEST_INCLUDES})
    target_link_libraries(test_xt_block_timing platutils_lib)
    # 9B: the driver's own tests (uncompiled on the ROS1 station; checked on Jazzy).
    ament_add_gtest(test_driver_accounting test/test_driver_accounting.cpp)
    target_include_directories(test_driver_accounting PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
    ament_add_gtest(test_config_defaults test/test_config_defaults.cpp)
    target_include_directories(test_config_defaults PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
    target_link_libraries(test_config_defaults ${HESAI_SDK_LINK_LIBS})
    ament_add_gtest(test_sdk_counters test/test_sdk_counters.cpp)
    target_include_directories(test_sdk_counters PRIVATE ${HESAI_XT_TEST_INCLUDES})
    target_link_libraries(test_sdk_counters platutils_lib)
    ament_add_gtest(test_gpu_frame_step test/test_gpu_frame_step.cpp)
    target_include_directories(test_gpu_frame_step PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
    foreach(_ros2_test test_ros2_stamp test_ros2_pointcloud_layout)
      ament_add_gtest(${_ros2_test} test/${_ros2_test}.cpp)
      target_include_directories(${_ros2_test} PRIVATE ${HESAI_SDK_INCLUDE_DIRS})
      target_link_libraries(${_ros2_test} ${HESAI_SDK_LINK_LIBS})
      ament_target_dependencies(${_ros2_test} rclcpp sensor_msgs std_msgs diagnostic_msgs
        builtin_interfaces)
      rosidl_target_interfaces(${_ros2_test} ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endforeach()
  endif()

  ament_package()

endif($ENV{ROS_VERSION} MATCHES "2")


