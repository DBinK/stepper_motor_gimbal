import time
import serial
from stepper import EmmMotor


class Gimbal:
    """
    云台控制类，使用两个EmmMotor电机分别控制水平和垂直方向的转动
    """

    def __init__(self, horizontal_port: str, vertical_port: str, horizontal_addr: int = 1, vertical_addr: int = 2,
                 baud_rate: int = 115200):
        """
        初始化云台对象
        :param horizontal_port: 水平电机串口端口
        :param vertical_port: 垂直电机串口端口
        :param horizontal_addr: 水平电机地址
        :param vertical_addr: 垂直电机地址
        :param baud_rate: 串口波特率
        """
        # 如果两个电机使用同一个串口
        if horizontal_port == vertical_port:
            self.shared_serial = serial.Serial(vertical_port, baud_rate, timeout=0.1)
            self.horizontal_motor = EmmMotor(horizontal_addr, self.shared_serial)
            self.vertical_motor = EmmMotor(vertical_addr, self.shared_serial)
        else:
            # 如果使用不同的串口
            self.shared_serial = None
            self.horizontal_motor = EmmMotor(horizontal_addr,
                                             serial.Serial(horizontal_port, baud_rate, timeout=0.1))
            self.vertical_motor = EmmMotor(vertical_addr, serial.Serial(vertical_port, baud_rate, timeout=0.1))

        # 云台状态
        self.horizontal_angle = 0.0
        self.vertical_angle = 0.0
        self.target_x = None
        self.target_y = None

        # 云台参数配置
        self.horiz_move_factor = 0.5  # 水平移动因子
        self.vert_move_factor = 0.5   # 垂直移动因子

        # 初始化电机
        self._init_motors()

    def _init_motors(self):
        """
        初始化电机参数
        """
        # 使能电机控制
        self.horizontal_motor.enable_control(state=True, snF=False)
        self.vertical_motor.enable_control(state=True, snF=False)
        time.sleep(0.1)

    def get_current_angles(self):
        """
        获取当前云台角度
        :return: (horizontal_angle, vertical_angle)
        """
        self.horizontal_angle = self.horizontal_motor.get_real_position()
        self.vertical_angle = self.vertical_motor.get_real_position()
        return self.horizontal_angle, self.vertical_angle

    def move_to_absolute_angle(self, horizontal_angle: float, vertical_angle: float):
        """
        绝对角度移动云台
        :param horizontal_angle: 水平角度
        :param vertical_angle: 垂直角度
        """
        # 计算需要移动的角度差
        current_horizontal, current_vertical = self.get_current_angles()
        
        horizontal_diff = horizontal_angle - current_horizontal
        vertical_diff = vertical_angle - current_vertical
        
        # 转换为脉冲数并控制电机
        self._move_motors_by_angle(horizontal_diff, vertical_diff)

    def move_by_angle(self, horizontal_diff: float, vertical_diff: float):
        """
        相对角度移动云台
        :param horizontal_diff: 水平角度变化量
        :param vertical_diff: 垂直角度变化量
        """
        self._move_motors_by_angle(horizontal_diff, vertical_diff)

    def _move_motors_by_angle(self, horizontal_angle_diff: float, vertical_angle_diff: float):
        """
        根据角度差移动电机
        :param horizontal_angle_diff: 水平角度差
        :param vertical_angle_diff: 垂直角度差
        """
        # 将角度转换为脉冲数 (这里假设1000脉冲=360度，需要根据实际电机参数调整)
        horizontal_pulses = int(abs(horizontal_angle_diff) * 1000 / 360)
        vertical_pulses = int(abs(vertical_angle_diff) * 1000 / 360)
        
        # 确定方向 (0: 正转, 1: 反转)
        horizontal_direction = 0 if horizontal_angle_diff >= 0 else 1
        vertical_direction = 0 if vertical_angle_diff >= 0 else 1
        
        # 控制水平电机
        if horizontal_pulses > 0:
            self.horizontal_motor.position_control(
                direction=horizontal_direction,
                velocity=1000,
                acceleration=100,
                pulses=horizontal_pulses,
                raF=False,
                snF=False
            )
        
        # 控制垂直电机
        if vertical_pulses > 0:
            self.vertical_motor.position_control(
                direction=vertical_direction,
                velocity=1000,
                acceleration=100,
                pulses=vertical_pulses,
                raF=False,
                snF=False
            )
        
        # 更新角度状态
        self.horizontal_angle += horizontal_angle_diff
        self.vertical_angle += vertical_angle_diff

    def auto_track_target(self, target_x: int, target_y: int, frame_width: int, frame_height: int):
        """
        自动跟踪目标
        :param target_x: 目标x坐标
        :param target_y: 目标y坐标
        :param frame_width: 图像宽度
        :param frame_height: 图像高度
        """
        # 计算图像中心点
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        # 计算偏差
        delta_x = target_x - center_x
        delta_y = target_y - center_y
        
        # 转换为角度调整量
        horizontal_adjust = delta_x * self.horiz_move_factor / center_x
        vertical_adjust = -delta_y * self.vert_move_factor / center_y  # y轴方向相反
        
        # 移动云台
        self.move_by_angle(horizontal_adjust, vertical_adjust)

    def stop(self):
        """
        立即停止云台运动
        """
        self.horizontal_motor.stop_now(snF=False)
        self.vertical_motor.stop_now(snF=False)

    def reset_position(self):
        """
        将当前位置设为零点
        """
        self.horizontal_motor.reset_current_position_to_zero()
        self.vertical_motor.reset_current_position_to_zero()
        self.horizontal_angle = 0.0
        self.vertical_angle = 0.0

    def close(self):
        """
        关闭串口连接
        """
        if self.shared_serial:
            try:
                self.shared_serial.close()
            except:
                pass
        else:
            try:
                self.horizontal_motor.ser.close()
            except:
                pass
            try:
                self.vertical_motor.ser.close()
            except:
                pass


# 使用示例
if __name__ == "__main__":
    # 创建云台对象 (根据实际串口配置修改)
    gimbal = Gimbal(horizontal_port='COM13', vertical_port='COM28')
    
    try:
        # 获取当前位置
        h_angle, v_angle = gimbal.get_current_angles()
        print(f"当前角度 - 水平: {h_angle:.1f}°, 垂直: {v_angle:.1f}°")
        
        # 相对移动
        gimbal.move_by_angle(30, 15)  # 水平右转30度，垂直上转15度
        time.sleep(2)
        
        # 绝对移动到指定角度
        gimbal.move_to_absolute_angle(45, 30)  # 移动到水平45度，垂直30度
        time.sleep(2)
        
        # 重置位置
        gimbal.reset_position()
        
    except Exception as e:
        print(f"运行出错: {e}")
    finally:
        # 停止并关闭
        gimbal.stop()
        gimbal.close()
        print("云台已停止并关闭连接")