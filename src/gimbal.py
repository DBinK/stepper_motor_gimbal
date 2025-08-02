import time

import loguru
import serial
from loguru import logger

from stepper import EmmMotor


class Gimbal:
    """
    云台控制类，使用两个EmmMotor电机分别控制水平和垂直方向的转动
    """

    def __init__(
        self,
        horizontal_port: str,
        vertical_port: str,
        horizontal_addr: int = 2,
        vertical_addr: int = 1,
        baud_rate: int = 115200,
    ):
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
            self.horizontal_motor = EmmMotor(
                horizontal_addr, serial.Serial(horizontal_port, baud_rate, timeout=0.1)
            )
            self.vertical_motor = EmmMotor(
                vertical_addr, serial.Serial(vertical_port, baud_rate, timeout=0.1)
            )

        # 云台状态
        self.horizontal_angle = 0.0
        self.vertical_angle = 0.0
        self.target_x = None
        self.target_y = None

        # 云台角度限位设置
        self.horizontal_min_angle = -180.0  # 水平最小角度限制
        self.horizontal_max_angle = 180.0   # 水平最大角度限制
        self.vertical_min_angle = -90.0     # 垂直最小角度限制
        self.vertical_max_angle = 90.0      # 垂直最大角度限制

        # 云台参数配置
        self.horiz_move_factor = 100.5  # 水平移动因子
        self.vert_move_factor = 100.5  # 垂直移动因子

        # 性能优化相关
        self.last_angle_update = 0.0  # 上次角度更新时间
        self.angle_cache_duration = 0.1  # 角度缓存时间（秒）

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

    def set_angle_limits(self, horizontal_min: float, horizontal_max: float, 
                         vertical_min: float, vertical_max: float):
        """
        设置云台角度限位
        :param horizontal_min: 水平最小角度
        :param horizontal_max: 水平最大角度
        :param vertical_min: 垂直最小角度
        :param vertical_max: 垂直最大角度
        """
        if horizontal_min >= horizontal_max:
            raise ValueError("水平最小角度必须小于最大角度")
        if vertical_min >= vertical_max:
            raise ValueError("垂直最小角度必须小于最大角度")
            
        self.horizontal_min_angle = horizontal_min
        self.horizontal_max_angle = horizontal_max
        self.vertical_min_angle = vertical_min
        self.vertical_max_angle = vertical_max

    def get_current_angles(self, force_update=False):
        """
        获取当前云台角度（带缓存优化）
        :param force_update: 是否强制更新，忽略缓存
        :return: (horizontal_angle, vertical_angle)
        """
        current_time = time.time()
        # 如果缓存有效且不强制更新，则直接返回缓存值
        if (not force_update and 
            (current_time - self.last_angle_update) < self.angle_cache_duration):
            return self.horizontal_angle, self.vertical_angle

        # 更新角度并记录更新时间
        self.horizontal_angle = self.horizontal_motor.get_real_position()
        self.vertical_angle = self.vertical_motor.get_real_position()
        self.last_angle_update = current_time
        return self.horizontal_angle, self.vertical_angle

    def move_to_absolute_angle(self, horizontal_angle: float, vertical_angle: float):
        """
        绝对角度移动云台
        :param horizontal_angle: 水平角度
        :param vertical_angle: 垂直角度
        """
        # 检查角度限位
        horizontal_angle = max(self.horizontal_min_angle, 
                              min(self.horizontal_max_angle, horizontal_angle))
        vertical_angle = max(self.vertical_min_angle, 
                            min(self.vertical_max_angle, vertical_angle))

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
        # 预测移动后的位置
        current_horizontal, current_vertical = self.get_current_angles()
        target_horizontal = current_horizontal + horizontal_diff
        target_vertical = current_vertical + vertical_diff

        # 检查是否会超出限位
        if (target_horizontal < self.horizontal_min_angle or 
            target_horizontal > self.horizontal_max_angle):
            logger.warning(f"水平角度 {target_horizontal:.2f}° 超出限制范围 "
                          f"[{self.horizontal_min_angle}, {self.horizontal_max_angle}]")
            # 限制在范围内
            target_horizontal = max(self.horizontal_min_angle, 
                                   min(self.horizontal_max_angle, target_horizontal))
            # 重新计算差值
            horizontal_diff = target_horizontal - current_horizontal

        if (target_vertical < self.vertical_min_angle or 
            target_vertical > self.vertical_max_angle):
            logger.warning(f"垂直角度 {target_vertical:.2f}° 超出限制范围 "
                          f"[{self.vertical_min_angle}, {self.vertical_max_angle}]")
            # 限制在范围内
            target_vertical = max(self.vertical_min_angle, 
                                 min(self.vertical_max_angle, target_vertical))
            # 重新计算差值
            vertical_diff = target_vertical - current_vertical

        self._move_motors_by_angle(horizontal_diff, vertical_diff)

    def _move_motors_by_angle(
        self, horizontal_angle_diff: float, vertical_angle_diff: float, raF: bool = False
    ):
        """
        根据角度差移动电机
        :param horizontal_angle_diff: 水平角度差
        :param vertical_angle_diff: 垂直角度差
        """
        # 将角度转换为脉冲数 (这里假设3200脉冲=360度，需要根据实际电机参数调整)
        horizontal_pulses = int(abs(horizontal_angle_diff) * 3200 / 360)
        vertical_pulses = int(abs(vertical_angle_diff) * 3200 / 360)

        print(f"水平角度差: {horizontal_angle_diff:.2f}°, 脉冲数: {horizontal_pulses}")
        print(f"垂直角度差: {vertical_angle_diff:.2f}°, 脉冲数: {vertical_pulses}")

        # 确定方向 (0: 正转, 1: 反转)
        horizontal_direction = 0 if horizontal_angle_diff >= 0 else 1
        vertical_direction = 0 if vertical_angle_diff >= 0 else 1

        # 控制水平电机
        if horizontal_pulses >= 0:
            self.horizontal_motor.position_control(
                direction=horizontal_direction,
                velocity=1000,
                acceleration=255,
                pulses=horizontal_pulses,
                raF=raF,
                snF=False,
            )

        time.sleep(0.005)  # 必须延时, 不然串口信号会乱

        # 控制垂直电机
        if vertical_pulses >= 0:
            self.vertical_motor.position_control(
                direction=vertical_direction,
                velocity=1000,
                acceleration=255,
                pulses=vertical_pulses,
                raF=raF,
                snF=False,
            )

        time.sleep(0.005)  # 必须延时, 不然串口信号会乱

        # 更新角度状态
        self.horizontal_angle += horizontal_angle_diff
        self.vertical_angle += vertical_angle_diff

        # 更新角度更新时间
        self.last_angle_update = time.time()

    def auto_track_target(
        self, target_x: int, target_y: int, frame_width: int, frame_height: int
    ):
        """
        自动跟踪目标（优化版本，提高性能）
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
        vertical_adjust = delta_y * self.vert_move_factor / center_y  # y轴方向相反

        # 移动云台（使用缓存的角度值提高性能）
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
        self.last_angle_update = time.time()  # 重置角度更新时间

    def close(self):
        """
        关闭串口连接
        """
        if self.shared_serial:
            try:
                self.shared_serial.close()
            except Exception as e:
                print(f"串口关闭出错: {e}")
        else:
            try:
                self.horizontal_motor.ser.close()
            except Exception as e:
                print(f"串口关闭出错: {e}")
            try:
                self.vertical_motor.ser.close()
            except Exception as e:
                print(f"串口关闭出错: {e}")


# 使用示例
if __name__ == "__main__":
    # 创建云台对象 (根据实际串口配置修改)
    # gimbal = Gimbal(horizontal_port="COM29", vertical_port="COM29")
    gimbal = Gimbal(horizontal_port="/dev/ttyS6", vertical_port="/dev/ttyS6")

    try:
        # 设置角度限位 (可选，默认为水平-180~180度，垂直-90~90度)
        gimbal.set_angle_limits(horizontal_min=-170, horizontal_max=170,
                               vertical_min=-80, vertical_max=80)

        # 获取当前位置
        h_angle, v_angle = gimbal.get_current_angles()
        print(f"当前角度 - 水平: {h_angle:.1f}°, 垂直: {v_angle:.1f}°")

        # 相对移动
        gimbal.move_by_angle(45, 30)  # 水平右转30度，垂直上转15度
        time.sleep(1)

        gimbal.move_by_angle(-45, -30)  # 水平右转30度，垂直上转15度
        time.sleep(1)

        gimbal.auto_track_target(100, 100, 720, 1280)
        time.sleep(1)

        # 绝对移动到指定角度
        gimbal._move_motors_by_angle(0, 0, True)  # 移动到水平0度，垂直0度
        time.sleep(2)

        # 重置位置
        # gimbal.reset_position()

    except Exception as e:
        print(f"运行出错: {e}")
    finally:
        # 停止并关闭
        gimbal.stop()
        gimbal.close()
        print("云台已停止并关闭连接")