uint8 INIT = 0                                      # 初始化状态  
uint8 SEARCHING_PARKING_SPACE = 1                   # 搜寻停靠点
uint8 MOVING_TO_PARKING_SPACE = 2                   # 前往停靠点
uint8 GENERATE_PATH = 3                             # 生成路径
uint8 THROUGH_POSES_AND_SEARCHING_PARKING_SPACE = 4 # 边走边找停靠点
uint8 WAITING = 5                                   # 等待车辆通过
uint8 FINAL = 99                                    # 结束时状态
uint8 state