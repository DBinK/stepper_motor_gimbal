import time
import serial
import struct

# 配置串口 (根据实际设备修改端口号)
# SERIAL_PORT1 = '/dev/ttyUSB0'  # Linux 示例，Windows 用 'COM3' 等
# SERIAL_PORT2 = '/dev/ttyUSB1'
SERIAL_PORT1 = 'COM11'  # Linux 示例，Windows 用 'COM3' 等
SERIAL_PORT2 = 'COM28'
BAUD_RATE = 115200

# 初始化串口
try:
    ser1 = serial.Serial(SERIAL_PORT1, BAUD_RATE, timeout=0.1)
    ser2 = serial.Serial(SERIAL_PORT2, BAUD_RATE, timeout=0.1)
    print(f"已连接串口: {SERIAL_PORT1} 和 {SERIAL_PORT2}")
except Exception as e:
    print(f"串口初始化失败: {e}")
    exit(1)

# 对照表 (与原始代码一致)
'''
    S_VER = 0      # 读取固件版本和对应的硬件版本
    S_RL = 1       # 读取相电阻和相电感
    S_PID = 2      # 读取PID参数
    S_VBUS = 3     # 读取总线电压
    S_CPHA = 5     # 读取相电流
    S_ENCL = 7     # 读取经过线性化校准后的编码器值
    S_TPOS = 8     # 读取电机目标位置角度
    S_VEL = 9      # 读取电机实时转速
    S_CPOS = 10    # 读取电机实时位置角度
    S_PERR = 11    # 读取电机位置误差角度
    S_FLAG = 13    # 读取使能/到位/堵转状态标志位
    S_Conf = 14    # 读取驱动参数
    S_State = 15   # 读取系统状态参数
    S_ORG = 16     # 读取正在回零/回零失败状态标志位
'''

def Emm_V5_Read_Sys_Params(addr, s):
    """读取驱动板参数"""
    func_codes = {
        'S_VER': 0x1F,
        'S_RL': 0x20,
        'S_PID': 0x21,
        'S_VBUS': 0x24,
        'S_CPHA': 0x27,
        'S_ENCL': 0x31,
        'S_TPOS': 0x33,
        'S_VEL': 0x35,
        'S_CPOS': 0x36,
        'S_PERR': 0x37,
        'S_FLAG': 0x3A,
        'S_ORG': 0x3B,
        'S_Conf': 0x42,
        'S_State': 0x43
    }
    
    if s not in func_codes:
        raise ValueError(f"无效的参数类型: {s}")
    
    cmd = bytearray([
        addr,           # 地址
        func_codes[s],  # 功能码
        0x6B            # 校验字节
    ])
    
    # 根据地址选择串口
    if addr == 1:
        ser1.write(cmd)
    elif addr == 2:
        ser2.write(cmd)
    else:
        raise ValueError(f"不支持的地址: {addr}")

def Emm_V5_Reset_CurPos_To_Zero(addr):
    """将当前位置清零"""
    cmd = bytearray([addr, 0x0A, 0x6D, 0x6B])
    _send_to_addr(addr, cmd)

def Emm_V5_Reset_Clog_Pro(addr):
    """解除堵转保护"""
    cmd = bytearray([addr, 0x0E, 0x52, 0x6B])
    _send_to_addr(addr, cmd)

def Emm_V5_Modify_Ctrl_Mode(addr, svF, ctrl_mode):
    """修改控制模式"""
    cmd = bytearray([
        addr,
        0x46,
        0x69,
        0x01 if svF else 0x00,
        ctrl_mode,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_En_Control(addr, state, snF):
    """使能电机控制"""
    cmd = bytearray([
        addr,
        0xF3,
        0xAB,
        0x01 if state else 0x00,
        0x01 if snF else 0x00,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_Vel_Control(addr, dir, vel, acc, snF):
    """速度控制"""
    cmd = bytearray([
        addr,
        0xF6,
        dir & 0xFF,
        (vel >> 8) & 0xFF,
        vel & 0xFF,
        acc & 0xFF,
        0x01 if snF else 0x00,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_Pos_Control(addr, dir, vel, acc, clk, raF, snF):
    """位置控制"""
    cmd = bytearray([
        addr,
        0xFD,
        dir & 0xFF,
        (vel >> 8) & 0xFF,
        vel & 0xFF,
        acc & 0xFF,
        (clk >> 24) & 0xFF,
        (clk >> 16) & 0xFF,
        (clk >> 8) & 0xFF,
        clk & 0xFF,
        0x01 if raF else 0x00,
        0x01 if snF else 0x00,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_Stop_Now(addr, snF):
    """立即停止"""
    cmd = bytearray([
        addr,
        0xFE,
        0x98,
        0x01 if snF else 0x00,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_Synchronous_motion(addr):
    """执行多机同步运动"""
    cmd = bytearray([addr, 0xFF, 0x66, 0x6B])
    _send_to_addr(addr, cmd)

def Emm_V5_Origin_Set_O(addr, svF):
    """设置单圈回零零点位置"""
    cmd = bytearray([
        addr,
        0x93,
        0x88,
        0x01 if svF else 0x00,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_Origin_Modify_Params(addr, svF, o_mode, o_dir, o_vel, o_tm, sl_vel, sl_ma, sl_ms, potF):
    """修改回零参数"""
    cmd = bytearray([
        addr,
        0x4C,
        0xAE,
        0x01 if svF else 0x00,
        o_mode & 0xFF,
        o_dir & 0xFF,
        (o_vel >> 8) & 0xFF,
        o_vel & 0xFF,
        (o_tm >> 24) & 0xFF,
        (o_tm >> 16) & 0xFF,
        (o_tm >> 8) & 0xFF,
        o_tm & 0xFF,
        (sl_vel >> 8) & 0xFF,
        sl_vel & 0xFF,
        (sl_ma >> 8) & 0xFF,
        sl_ma & 0xFF,
        (sl_ms >> 8) & 0xFF,
        sl_ms & 0xFF,
        0x01 if potF else 0x00,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_Origin_Trigger_Return(addr, o_mode, snF):
    """触发回零"""
    cmd = bytearray([
        addr,
        0x9A,
        o_mode & 0xFF,
        0x01 if snF else 0x00,
        0x6B
    ])
    _send_to_addr(addr, cmd)

def Emm_V5_Origin_Interrupt(addr):
    """强制中断回零"""
    cmd = bytearray([addr, 0x9C, 0x48, 0x6B])
    _send_to_addr(addr, cmd)

def _send_to_addr(addr, cmd):
    """内部函数：根据地址选择串口发送"""
    if addr == 1:
        ser1.write(cmd)
    elif addr == 2:
        ser2.write(cmd)
    else:
        raise ValueError(f"不支持的地址: {addr}")

def Emm_V5_Receive_Data(ser, timeout=0.1):
    """接收串口数据（带超时）"""
    start_time = time.time()
    buffer = bytearray()
    
    while (time.time() - start_time) < timeout:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
            start_time = time.time()  # 重置超时计时器
        time.sleep(0.001)  # 小延时避免CPU占用过高
    
    # 转换为十六进制字符串
    hex_data = ' '.join([f'{b:02x}' for b in buffer])
    return hex_data, len(buffer)

def Real_time_location():
    """获取两个电机的实时位置"""
    # 读取电机1位置
    Emm_V5_Read_Sys_Params(1, 'S_CPOS')
    time.sleep(0.001)  # 1ms 延时
    
    # 读取电机2位置
    Emm_V5_Read_Sys_Params(2, 'S_CPOS')
    time.sleep(0.001)
    
    # 接收数据
    data1, len1 = Emm_V5_Receive_Data(ser1)
    data2, len2 = Emm_V5_Receive_Data(ser2)
    
    # 解析数据
    pos1 = _parse_position_data(data1, 1)
    pos2 = _parse_position_data(data2, 2)
    
    print(f'Motor1: {pos1:.1f}, Motor2: {pos2:.1f}')
    return pos1, pos2

def _parse_position_data(hex_data, expected_addr):
    """解析位置数据"""
    if not hex_data:
        return 0.0
    
    parts = hex_data.split()
    if len(parts) < 7:
        return 0.0
    
    try:
        # 验证地址和指令
        addr = int(parts[0], 16)
        cmd = int(parts[1], 16)
        if addr != expected_addr or cmd != 0x36:
            return 0.0
        
        # 提取4字节位置数据 (大端序)
        pos_bytes = bytes([
            int(parts[3], 16),
            int(parts[4], 16),
            int(parts[5], 16),
            int(parts[6], 16)
        ])
        pos_value = struct.unpack('>I', pos_bytes)[0]
        
        # 转换为角度 (360°/65536)
        angle = (pos_value * 360.0) / 65536.0
        
        # 检查方向标志
        if int(parts[2], 16) != 0:
            angle = -angle
            
        return angle
    except (IndexError, ValueError, struct.error):
        return 0.0

# ======================
# 示例使用
# ======================
if __name__ == "__main__":
    try:
        # 使能电机1 (地址1)
        # Emm_V5_En_Control(1, True, False)
        # time.sleep(0.1)
        
        # 使能电机2 (地址2)
        Emm_V5_En_Control(2, True, False)
        time.sleep(0.1)
        
        # 位置控制示例 (电机1: 顺时针, 1000RPM, 加速度50, 2000脉冲)
        Emm_V5_Pos_Control(2, 0, 4000, 0, 100, False, False)
        time.sleep(0.1)
        
        # # 速度控制示例 (电机2: 逆时针, 500RPM)
        # Emm_V5_Vel_Control(2, 1, 500, 30, False)
        
        # 持续获取位置
        # print("实时位置 (Ctrl+C 退出):")
        # while True:
        #     Real_time_location()
        #     time.sleep(0.1)  # 100ms 刷新率
            
    except KeyboardInterrupt:
        print("\n停止程序")
        # 停止所有电机
        Emm_V5_Stop_Now(1, False)
        Emm_V5_Stop_Now(2, False)
    finally:
        ser1.close()
        ser2.close()