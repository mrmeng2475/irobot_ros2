# 此代码是基于ros2 foxy框架所搭建的，其余框架下均徐修改

# 设置静态ip
sudo ifconfig wlp2s0 192.168.1.100 netmask 255.255.255.0
# ------------------------------------------ # 
# 关节范围
## 左臂           # 右臂
(-3.14, 3.14) |  (-3.14, 3.14)
(-3.14, 0.15) |  (-0.15, 3.14)
(-3.14, 0.95) |  (-0.95, 3.14)
(-1.57, 2.0 ) |  (-1.57, 2.0 ) 
(-3.14, 3.14) |  (-3.14, 3.14)  
(-1.57, 1.57) |  (-1.57, 1.57)
(-3.14, 3.14) |  (-3.14, 3.14)


# ------------------------------------------ # 
# 机械臂旋转方向与urdf对比(单臂)

1：反
2：正 
3：正
4：反
5：反
6：正
7：正
夹爪关节： 正是打开，负是关闭

# ------------------------------------------ #
# 简单的机械臂关节控制
## 终端1
. source.sh
ros2 run mintasa_ros_control joint_controller
## 终端2
. source.sh
ros2 topic pub --once /joint_cmd sensor_msgs/msg/JointState "{name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7','joint8'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

ros2 topic pub --once /joint_cmd sensor_msgs/msg/JointState "{name: ['joint1', 'joint5'], position: [1.0, 1]}"

# ------------------------------------------ #
# 双臂的键盘遥操控制(目前是单臂)

### 终端1 (仿真模式下可以不用这个终端)
. source.sh
ros2 run mintasa_ros_control R_controller

### 终端2 (启动rviz仿真，记得把rviz的joint_publisher关节控制关掉)
. source.sh
ros2 launch irobot_description display.launch.py

### 终端3 
. source.sh
ros2 run dual_arm_ik joint_state_aggregator_node 

### 终端4
. source.sh
ros2 run dual_arm_ik ik_node ### 注意！！！！ 真机有桌子不能运行这个节点！！！！！！

### 终端5
. source.sh
ros2 run dual_arm_teleop teleop_node


# ------------------------------------------ #
# 夹爪的位置和力矩模式切换(demo测试)
### 终端1 
. source.sh
ros2 run mintasa_ros_control clip_controller

### 终端2 
. source.sh
ros2 topic pub /clip_cmd irobot_interfaces/msg/ClipCommand "{mode: 1, value: 0.5}" -1 # 位置控制
ros2 topic pub /clip_cmd irobot_interfaces/msg/ClipCommand "{mode: 2, value: -0.8}" -1 # 电流控制

# ------------------------------------------ #
# 跑抓取固定轨迹demo

### 终端1 (仿真模式下可以不用这个终端)
. source.sh
ros2 launch dual_arm_ik controllers.launch.py ##最后关，保证开启，不然再开会回零点，撞桌子！！！

### 终端2 (启动rviz仿真，记得把rviz的joint_publisher关节控制关掉)
. source.sh
ros2 launch irobot_description display.launch.py

### 终端3 
. source.sh
ros2 run dual_arm_ik ik_posture_node 

### 终端4
. source.sh
ros2 run irobot_trajectory irobot_plan2

### 终端5
. source.sh
ros2 run mintasa_ros_control R_whole_control

# ------------------------------------------ #
# 跑抓取双臂固定轨迹demo(mujoco仿真)

### 终端1 (仿真模式下可以不用这个终端)
. source.sh
ros2 launch dual_arm_ik controllers.launch

### 终端2 (启动rviz仿真，记得把rviz的joint_publisher关节控制关掉)
. source.sh
ros2 launch irobot_description display.launch

### 终端3 
. source.sh
ros2 run dual_arm_ik ik_posture_node 

### 终端4
. source.sh
ros2 run irobot_trajectory irobot_plan3




