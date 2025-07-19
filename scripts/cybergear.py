from cybergear_can_control import CANMotorController
import time

if __name__ == "__main__":
    #实例化can控制器
    _controller = CANMotorController()
    #初始化can通讯
    controller = _controller.can_init()
    # 初始化电机id
    motor_id =127
    # 修改电机id
    # controller.set_id(motor_id,x)
    #使能电机
    controller.start_cmd(motor_id)
    time.sleep(2)
    controller.set_zero_cmd(motor_id)
    time.sleep(1)
    # controller.shutdown()
    #循环监听电机数据
    # try: 
    #     while True:
    #         controller.listen_can_feedback(motor_id)
    #         print(controller.data_array)
    # except KeyboardInterrupt:
    #     controller.shutdown()
    #力矩控制
    controller.send_command(motor_id,"run",torque=-0.5,vel_limit=0,pos=0,kp=0,kd=0)
    print("力矩控制模式")
    time.sleep(2)
    #设置电机零位
    controller.set_zero_cmd(motor_id)
    print("设置当前位置为0位")
    #位置控制
    pos=0.5
    print(f"位置控制,目标{pos}rad")
    controller.send_command(motor_id,"run",torque=0,vel_limit=0,pos=pos,kp=10,kd=1)
    time.sleep(2)
    #插值位置控制
    print("插值位置控制")
    controller.interpolating_control(motor_id,1,500)
    time.sleep(2)
    #回零位模式
    controller.back2zero(motor_id)
    #阻尼模式
    controller.damping_cmd(motor_id,kd=4)

    time.sleep(3)
    #失能电机
    controller.stop_cmd(motor_id)
    time.sleep(3)
    #使能电机
    controller.start_cmd(motor_id)
    time.sleep(3)
    #失能所有电机并关闭can通讯
    controller.shutdown()



