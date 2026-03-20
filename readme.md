# 此代码是基于ros2 foxy框架所搭建的，其余框架下均徐修改

## 设置静态ip
sudo ifconfig wlp2s0 192.168.1.100 netmask 255.255.255.0
## ------------------------------------------ # 
### 关节范围
```
# 左臂           # 右臂
(-3.14, 3.14) |  (-3.14, 3.14)
(-3.14, 0.15) |  (-0.15, 3.14)
(-3.14, 0.95) |  (-0.95, 3.14)
(-1.57, 2.0 ) |  (-1.57, 2.0 ) 
(-3.14, 3.14) |  (-3.14, 3.14)  
(-1.57, 1.57) |  (-1.57, 1.57)
(-3.14, 3.14) |  (-3.14, 3.14)
```

### ------------------------------------------ # 
### 机械臂旋转方向与urdf对比(双臂)
```
 右臂  |   左臂
1：反  | 11:反      
2：正  | 12:正
3：反  | 13:反
4：正  | 14:反
5：反  | 15:反
6：反  | 16:反
7：正  | 17:正
夹爪关节： 正是打开，负是关闭
```
### ------------------------------------------ #
# 简单的机械臂关节控制
## 终端1
```bash
. source.sh
ros2 run mintasa_ros_control joint_controller
```
## 终端2
```bash
. source.sh
ros2 topic pub --once /joint_cmd sensor_msgs/msg/JointState "{name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7','joint8'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

```bash
ros2 topic pub --once /joint_cmd sensor_msgs/msg/JointState "{name: ['joint1', 'joint5'], position: [1.0, 1]}"
```
## ------------------------------------------ ##
## 双臂的键盘遥操控制(目前是单臂)

### 终端1 (仿真模式下可以不用这个终端)
```bash
. source.sh
ros2 run mintasa_ros_control R_controller
```
### 终端2 (启动rviz仿真，记得把rviz的joint_publisher关节控制关掉)
```bash
. source.sh
ros2 launch irobot_description display.launch.py
```
### 终端3 
```bash
. source.sh
ros2 run dual_arm_ik joint_state_aggregator_node 
```
### 终端4
```bash
. source.sh
ros2 run dual_arm_ik ik_node ### 注意！！！！ 真机有桌子不能运行这个节点！！！！！！
```
### 终端5
```bash
. source.sh
ros2 run dual_arm_teleop teleop_node
```

## ------------------------------------------ #
## 夹爪的位置和力矩模式切换(demo测试)
### 终端1 
```bash
. source.sh
ros2 run mintasa_ros_control clip_controller
```

### 终端2 
```bash
. source.sh
ros2 topic pub /clip_cmd irobot_interfaces/msg/ClipCommand "{mode: 1, value: 0.5}" -1 # 位置控制
ros2 topic pub /clip_cmd irobot_interfaces/msg/ClipCommand "{mode: 2, value: -0.8}" -1 # 电流控制
```
## ------------------------------------------ #
## 跑抓取固定轨迹demo

### 终端1 (仿真模式下可以不用这个终端)
```bash
. source.sh
ros2 launch dual_arm_ik controllers.launch.py ##最后关，保证开启，不然再开会回零点，撞桌子！！！
```

### 终端2 (启动rviz仿真，记得把rviz的joint_publisher关节控制关掉)
```bash
. source.sh
ros2 launch irobot_description display.launch.py
```

### 终端3
```bash
. source.sh
ros2 run mintasa_ros_control R_whole_control
```

### 终端4
```bash
. source.sh
ros2 run dual_arm_ik ik_posture_node 
```

### 终端5
```bash
. source.sh
ros2 run irobot_trajectory irobot_plan2
```


## ------------------------------------------ #
## 跑抓取双臂固定轨迹demo(mujoco仿真)

### 终端1 (仿真模式下可以不用这个终端)

```bash
. source.sh
ros2 launch dual_arm_ik controllers.launch
```

### 终端2 (启动rviz仿真，记得把rviz的joint_publisher关节控制关掉)
```bash
. source.sh
ros2 launch irobot_description display.launch
```

### 终端3 
```bash
. source.sh
ros2 run dual_arm_ik ik_posture_node 
```


### 终端4
```bash
. source.sh
ros2 run irobot_trajectory irobot_plan3
```

# 跑抓取双臂物体识别抓取(注意机器人每次启动后电机要全部重新标零)

### 终端1 (启动所有数据传输节点，包括仿真和真机，注意这个其它关完才能关掉这个节点)
```bash
. source.sh
ros2 launch dual_arm_ik controllers.launch
```

### 终端2 (启动相机摄像头)
```bash
. source.sh
ros2 ros2 run irobot_camera camera_depth_node
```
### 终端3 (启动机器人所有关节电机——电机启动完之后再启动下面节点)
```bash
. source.sh
ros2 run mintasa_ros_control irobot_control 
```

### 终端4 (启动计算识别计算物体在机器人坐标系下的服务)
```bash
. source.sh
ros2 run irobot_camera head_track_service_node 
```

### 终端5 (这个节点是获取所有要的物体在机器人坐标系下的坐标)
```bash
. source.sh
ros2 run irobot_camera object_position_node 
```

### 终端6 (控制机器人双臂逆解)
```bash
. source.sh
ros2 run dual_arm_ik ik_posture_node 
```

### 终端7 (将物体坐标规划进抓取点)
```bash
. source.sh
ros2 run irobot_trajectory irobot_plan4
```
