# 指定最低要求的CMake版本
cmake_minimum_required(VERSION 3.5)

# 定义项目/ROS2包的名称
project(mintasa_ros_control)

# 查找这个包所需要的依赖 (新增了 sensor_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irobot_interfaces REQUIRED)

###########################################################
## --- Mintasa SDK 集成 ---
###########################################################

# 设置SDK的路径
set(MINTASA_SDK_PATH ${PROJECT_SOURCE_DIR}/third_party/mintasa/sdk)

# 将SDK的头文件目录也添加到搜索路径中
include_directories(
  include
  ${MINTASA_SDK_PATH}/include
  ${MINTASA_SDK_PATH}/include/asio
  ${MINTASA_SDK_PATH}/include/serial/include
)

# 添加SDK的库文件目录
link_directories(${MINTASA_SDK_PATH}/lib/linux_x86_64)

# 添加必要的编译选项
add_compile_options(-std=c++17 -fPIC -pthread -DASIO_STANDALONE -DASIO_HAS_STD_CHRONO)

###########################################################
## --- 结束 Mintasa SDK 集成 ---
###########################################################

# --- 为 R_whole_control 定义 --- 
add_executable(R_whole_control src/R_whole_control.cpp)

# 为 R_whole_control 链接ROS2的依赖
ament_target_dependencies(R_whole_control
  rclcpp
  std_msgs
  sensor_msgs
  irobot_interfaces
)

# 为 R_whole_control 链接Mintasa SDK的核心库
target_link_libraries(R_whole_control
  ActuatorController
  pthread
)

# 定义joint_controller节点可执行文件
add_executable(joint_controller src/joint_controller.cpp)

# 为您的节点链接ROS2的依赖 (新增了 sensor_msgs)
ament_target_dependencies(joint_controller
  rclcpp
  std_msgs
  sensor_msgs
)

# 链接Mintasa SDK的核心库和pthread线程库
target_link_libraries(joint_controller
  ActuatorController
  pthread
)

# --- 为 R_controller 定义 ---
add_executable(R_controller src/R_controller.cpp)

# 为 R_controller 链接ROS2的依赖 (假设它和joint_controller有相同的依赖)
ament_target_dependencies(R_controller
  rclcpp
  std_msgs
  sensor_msgs
)

# 为 R_controller 链接Mintasa SDK的核心库
target_link_libraries(R_controller
  ActuatorController
  pthread
)

# --- 为 clip_controller 定义 --- 
add_executable(clip_controller src/clip_controller.cpp)

# 为 clip_controller 链接ROS2的依赖
ament_target_dependencies(clip_controller
  rclcpp
  std_msgs
  sensor_msgs
  irobot_interfaces
)

# 为 clip_controller 链接Mintasa SDK的核心库
target_link_libraries(clip_controller
 
 ActuatorController
  pthread
)

# --- 为 head_controller 定义 ---
add_executable(head_controller src/head_controller.cpp)

# 为 head_controller 链接ROS2的依赖
ament_target_dependencies(head_controller
  rclcpp
  std_msgs
  sensor_msgs
  irobot_interfaces
)

# 为 head_controller 链接Mintasa SDK的核心库
target_link_libraries(head_controller
  ActuatorController
  pthread
)



# 安装可执行文件
install(TARGETS
  joint_controller
  R_controller
  head_controller
  clip_controller
  R_whole_control
  DESTINATION lib/${PROJECT_NAME}
)

# 安装第三方库文件到 install/lib 目录
install(FILES
    ${MINTASA_SDK_PATH}/lib/linux_x86_64/libActuatorController.so
    DESTINATION lib
)

# ament_package() 是构建ROS2包所必需的
ament_package()