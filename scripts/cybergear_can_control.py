import argparse
import os
import can
import struct
import sys
import time
from typing import Dict
import math
import numpy as np
from datetime import datetime


class MotorConfig:
    interpolation_steps = 500
    limit_torque = 4
    kp = 10
    kd = 1 
    

class CANMotorController:
    # 控制指令前缀
    START_CMD_PREFIX = 0x03000000
    RUN_CMD_PREFIX = 0x017FFF00
    STOP_CMD_PREFIX = 0x04000000
    LISTEN_CMD_PREFIX = 0x17000000
    SET_ZERO_CMD_PREFIX = 0x06000000
    SET_ID_CMD_PREFIX = 0x07000000
    WRITE_PARAM_CMD_PREFIX = 0x12000000
    # 反馈帧ID前缀
    FEEDBACK_PREFIX = 0x02000000
    FEEDBACK_PREFIX1 = 0x02800000
    
    def __init__(self, channel: str = 'can0', bitrate: int = 1000000):
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.running = False
        self.data_array = [0.0] * 3  # 初始化数据数组
        self.motor_state = False
        self.motor_start_set=set()
        self.config = MotorConfig
        #全局变量-------------------------------------------------------------
    def can_init(self):
        """配置CAN接口并连接总线"""
        try:
            # 设置CAN接口sudo ip link set can0 up type can bitrate 1000000
            # restart-ms 100 berr-reporting on
            os.system(
                f'sudo ip link set can0 type can bitrate 1000000')
            os.system(f'sudo ip link set can0 type can restart-ms 10')
            os.system(f'sudo ip link set can0 txqueuelen 4000000')
            os.system(f'sudo ifconfig can0 up')
            # 创建CAN总线实例（设置过滤器接收反馈帧）
            self.bus = can.Bus(
                channel=self.channel,
                interface='socketcan',
                receive_own_messages=False,
                can_filters=[
                    {"can_id": self.FEEDBACK_PREFIX,
                        "can_mask": 0x1FF00000, "extended": True},
                    {"can_id": self.FEEDBACK_PREFIX1, "can_mask": 0x1FF00000, "extended": True}
                ]
            )
            self.running = True
            print("can通讯初始化成功")
            return self
        except Exception as e:
            self.shutdown()
            raise RuntimeError(f"CAN初始化失败: {e}")

    def shutdown(self):#关闭所有电机和can通讯
        """关闭CAN接口"""
        for motor in list(self.motor_start_set):
            self.stop_cmd(motor)
        self.running = False
        time.sleep(2)
        self.bus.shutdown()
        #os.system(f'sudo ifconfig {self.channel} down')
        print("关闭can通讯")

    @staticmethod
    def _embed_torque(base_id: int, torque: float) -> int:
        """将力矩值编码到CAN ID中"""
        if not (-12.0 <= torque <= 12.0):
            raise ValueError("力矩值超出范围 (-12.0 ~ 12.0 Nm)")
        mapped = int((torque + 12.0) * (65535.0 / 24.0))
        return (base_id & 0xFF0000FF) | ((mapped & 0xFFFF) << 8)
    def generate_motor_command(self,
        torque: float,      # -12到12牛米
        motor_id: int,      # 0-255
        angle: float,       # -4π到4π
        angular_velocity: float,  # -30到30rad/s
        kp: float,          # 0-500
        kd: float           # 0-5
) :
    # 参数边界约束
        torque = max(min(torque, 12.0), -12.0)
        angle = max(min(angle, 4*math.pi), -4*math.pi)
        angular_velocity = max(min(angular_velocity, 30.0), -30.0)
        kp = max(min(kp, 500.0), 0.0)
        kd = max(min(kd, 5.0), 0.0)
        motor_id = max(min(motor_id, 255), 0)

    # 数据映射到0-65535范围
        def map_range(value, in_min, in_max):
            return int((value - in_min) * 0xFFFF / (in_max - in_min))

        torque_mapped = map_range(torque, -12, 12)
        angle_mapped = map_range(angle, -4*math.pi, 4*math.pi)
        av_mapped = map_range(angular_velocity, -30, 30)
        kp_mapped = map_range(kp, 0, 500)
        kd_mapped = map_range(kd, 0, 5)

        # 转换为十六进制字符串
        hex_parts = [
            "01",  # 固定前缀
            f"{torque_mapped:04x}",  # 力矩
            f"{motor_id:02x}",       # 电机ID
            f"{angle_mapped:04x}",  # 角度
            f"{av_mapped:04x}",     # 角速度
            f"{kp_mapped:04x}",     # KP参数
            f"{kd_mapped:04x}"      # KD参数
        ]
        can_id = bytes.fromhex("".join(hex_parts[0:3])) # 合并CAN ID
        data=bytes.fromhex("".join(hex_parts[3:]))
        return can_id, data

    def pos_mode_vel_limit(self, vel_limit: float=30.0):
        # 映射到0-2³²-1范围
        mapped = int(vel_limit * (2**16 - 1) / 30)
        # 转换为8位大写十六进制（自动补零）
        return format(mapped, '04X')
    def ieee754_to_hex(self,num, data_type='float'):
        # 根据数据类型选择格式符
        format_map = {
            'int': ('<I', 8),     # 4字节整数
            'short': ('<H', 4),   # 2字节短整数
            'float': ('<f', 8),   # 4字节浮点数
            'double': ('<d', 16)  # 8字节双精度
        }
        fmt, hex_len = format_map[data_type]
        # 打包为小端字节序二进制
        packed = struct.pack(fmt, num)
        return packed
    def start_cmd(self,motor_id):
        self.motor_start_set.add(motor_id)
        self.send_command(motor_id,"start")
        print(f"使能电机{motor_id}")
        self.motor_state = True
    def set_id(self,motor_id,set_id):
        print(f"将电机id从{motor_id}设置为{set_id}")
        self.send_command(motor_id,"set_id",set_id = set_id)
        time.sleep(1)
    def damping_cmd(self,motor_id,kd):
        self.send_command(motor_id,"run",torque=0,vel_limit=0,pos=0,kp=0,kd=kd)
        print("进入阻尼模式")
    def stop_cmd(self,motor_id):
        self.send_command(motor_id,"stop")
        self.motor_start_set.discard(motor_id)
        print(f"失能电机{motor_id}")
        self.motor_state = False
    def back2zero(self,motor_id):
        if not self.motor_state:
            self.start_cmd(motor_id)
        self.listen_can_feedback(motor_id)  # 获取电机角度
        print(f"从当前位置{self.data_array[0]}回零位")
        current_q = self.data_array[0] # 获取当前关节角度
        steps = self.config.interpolation_steps
        interpolated_angles = self.generate_interpolated_angles(current_q,0,steps)
        # print(interpolated_angles)
        for j in interpolated_angles:
            self.send_command(motor_id, "run", torque=0, vel_limit=0, pos=j,kp=self.config.kp,kd=self.config.kd)
            time.sleep(0.005)
        self.flush_buffer()
    def set_zero_cmd(self,motor_id):
        print("设置当前位置为0")
        self.send_command(motor_id, "set_zero")
        self.flush_buffer()
        time.sleep(1)
    
    def generate_interpolated_angles(self,motor_id,end,steps):
        self.listen_can_feedback(motor_id)
        start = self.data_array[0]
        # 生成线性插值角度
        interpolated_angles = np.linspace(start,end, steps)
        return interpolated_angles
    def interpolating_control(self,motor_id,end,steps):
        self.listen_can_feedback(2)
        print(f"当前位置是{self.data_array[0]}rad，目标位置是{end}rad")
        interpolated_angles = self.generate_interpolated_angles(motor_id,end,steps)
        for angle in interpolated_angles:
            self.send_command(motor_id,"run",torque=0, vel_limit=0, pos=angle,kp=self.config.kp,kd=self.config.kd)
            time.sleep(0.005)
    def parse_feedback(self, data: bytes) -> Dict:
        """解析电机反馈数据"""
        if len(data) != 8:  # 严格校验8字节长度[3](@ref)
            raise ValueError("无效反馈数据长度")

        # 角度解析（0-1字节，大端16位无符号）
        angle_raw = struct.unpack('>H', data[0:2])[0]
        angle = (-4 * math.pi) + (angle_raw / 65535) * \
            8 * math.pi  # 线性映射公式
        # 角速度解析（2-3字节，大端16位无符号）
        vel_raw = struct.unpack('>H', data[2:4])[0]
        angular_velocity = ((vel_raw / 65535) * 60) - \
            30  # 映射到-30~30 rad/s
        # 力矩解析（4-5字节，大端16位无符号）
        torque_raw = struct.unpack('>H', data[4:6])[0]
        torque = ((torque_raw / 65535) * 24) - 12  # 映射到-12~12 Nm
        # 温度解析（6-7字节，大端16位无符号）
        temp_raw = struct.unpack('>H', data[6:8])[0]
        temperature = temp_raw / 10.0  # 实际温度值

        return {
            "angle": angle,
            "angular_velocity": angular_velocity,
            "torque": torque,
            "temperature": temperature
        }
    def parse_input(self, data: bytes) -> Dict:
        """解析输入数据"""
        if len(data) != 8:  # 严格校验8字节长度[3](@ref)
            raise ValueError("无效输入数据长度")

        # 角度解析（0-1字节，大端16位无符号）
        angle_raw = struct.unpack('>H', data[0:2])[0]
        angle = (-4 * math.pi) + (angle_raw / 65535) * \
            8 * math.pi  # 线性映射公式
        # 角速度解析（2-3字节，大端16位无符号）
        vel_raw = struct.unpack('>H', data[2:4])[0]
        angular_velocity = ((vel_raw / 65535) * 60) - \
            30  # 映射到-30~30 rad/s
        # kp解析（4-5字节，大端16位无符号）
        kp_raw = struct.unpack('>H', data[4:6])[0]
        kp = ((kp_raw / 65535) * 500)  # 映射到0~500
        
        # kd解析（6-7字节，大端16位无符号）
        kd_raw = struct.unpack('>H', data[6:8])[0]
        kd = ((kd_raw / 65535) * 5)  # 映射到0~5
        

        return {
            "angle": angle,
            "angular_velocity": angular_velocity,
            "kp": kp,
            "kd": kd
        }

    
    def send_command(self, motor_id: int, action: str, set_id: int = 127,torque: float = 0.0, vel_limit: float = 0, pos: float = 0.0,kp: float = 5.0, kd: float = 0.5,torque_limit: float = 10):
        """发送控制命令"""
        motor_id = int(motor_id)
        if action == "start":
            can_id = self.START_CMD_PREFIX | motor_id
            data = bytes(8)
        elif action =="set_id":
            if self.motor_state == True:
                self.shutdown()
                raise Exception("请在电机未使能状态设置id！！！")
            else:
                can_id = self.SET_ID_CMD_PREFIX | motor_id | set_id <<16
                data = bytes(8)
        elif action == "set_torque_limit":
            can_id = self.WRITE_PARAM_CMD_PREFIX | motor_id 
            data = bytearray(8)
            data[0] = 0x0B
            data[1] = 0x70
            data[2] = 0x00
            data[3] = 0x00
            data[4:8] = self.ieee754_to_hex(num=torque_limit)
            data = bytes(data)
        elif action == "run":
            can_id,data = self.generate_motor_command(torque=torque, motor_id=motor_id, angle=pos, angular_velocity=vel_limit, kp=kp, kd=kd)
            can_id = int(can_id.hex(),16) | motor_id
            parser_input =self.parse_input(data)
            
        elif action == "stop":
            can_id = self.STOP_CMD_PREFIX | motor_id
            data = bytes(8)
        elif action == "listen":
            can_id = self.LISTEN_CMD_PREFIX | motor_id
            data = bytearray(8)
            data[0] = 0x70
            data[1] = 0x0A
            data = bytes(data)
        elif action == "set_zero":
            can_id = self.SET_ZERO_CMD_PREFIX | motor_id
            data = bytearray(8)
            data[0] = 0x01
            data[1] = 0x01
            data = bytes(data)
        else:
            raise ValueError("无效动作类型 (支持: start/run/stop)")
        # 创建并发送消息
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        # print(msg)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            raise RuntimeError(f"CAN发送失败: {e}")
    
    def  flush_buffer(self):
        """清空CAN接收缓冲区"""
        try:
            if self.bus:
                start_clear_time = time.monotonic()
                while True:
                    msg = self.bus.recv(timeout=0.0002)
                    if msg is None:
                        clear_time=time.monotonic()
                        print(f"缓冲区已清空 | 耗时{clear_time-start_clear_time:.6f}s")
                        
                        break
        except Exception as e:
            print(f"清空缓冲区时发生错误: {e}")
    def listen_can_feedback(self, motor_id,timeout=0.002):#至少0.2ms才能接受到can消息，不发送命令直接接收电机数据，这依赖于有一个线程频繁发送电机命令。
        feedback_data = []
        self.flush_buffer()
        try:
            if self.bus is not None:
                for i in range(3):
                    self.send_command(motor_id,"listen")
                    msg = self.bus.recv(timeout=timeout)
                    #print("message received",msg)
                    if msg is not None and (msg.arbitration_id >> 24 == 0x2):
                        feedback = self.parse_feedback(msg.data)
                        #print("feedback",feedback)
                        self.data_array[0] = feedback['angle']
                        self.data_array[1] = feedback['angular_velocity']
                        self.data_array[2] = feedback['torque']
                        return self.data_array
                    else:
                        print("no message received")
                        return None
            else:
                print("CAN总线未初始化")
                return None
        except Exception as e:
            print(f"监听 CAN 总线时发生错误: {e}")
       

    
    
   
    

    
    

    
