# 用于选择控制哪个夹爪的常量
uint8 GRIPPER_RIGHT = 0
uint8 GRIPPER_LEFT = 1

# 用于定义指令是打开还是关闭的常量
uint8 COMMAND_CLOSE = 0
uint8 COMMAND_OPEN = 1

# 这个指令是针对哪个夹爪 (使用上面的常量 GRIPPER_RIGHT 或 GRIPPER_LEFT)
uint8 gripper_select

# 要执行的指令 (使用上面的常量 COMMAND_CLOSE 或 COMMAND_OPEN)
uint8 command
