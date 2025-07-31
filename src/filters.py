
import numpy as np
from pykalman import KalmanFilter
from pykalman.sqrt import CholeskyKalmanFilter
import time

class CoordinateKalmanFilter:
    """用于实时坐标滤波的Kalman滤波器封装类"""
    
    def __init__(self, fps=30, initial_samples_needed=20, reset_timeout=1.0):
        """
        初始化Kalman滤波器
        
        参数:
            fps: 视频帧率，用于计算时间步长
            initial_samples_needed: 需要的初始样本数量，如果为None则跳过EM训练
            reset_timeout: 无检测时重置滤波器的超时时间(秒)
        """
        self.fps = fps
        self.dt = 1.0 / fps
        self.initial_samples_needed = initial_samples_needed
        self.reset_timeout = reset_timeout
        
        # 初始化状态
        self.kalman_initialized = False
        self.initial_samples = []
        self.filtered_state_mean = None
        self.filtered_state_covariance = None
        self.last_detection_time = time.time()
        self.kf = self._initialize_kalman_filter()
        
    def _initialize_kalman_filter(self):
        """创建并初始化KalmanFilter对象"""
        # 状态转移矩阵 - 匀速运动模型 [x, vx, y, vy]
        transition_matrix = np.array([
            [1, self.dt, 0, 0],
            [0, 1,  0, 0],
            [0, 0,  1, self.dt],
            [0, 0,  0, 1]
        ])
        
        # 观测矩阵 - 直接测量位置
        observation_matrix = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])
        
        # 创建KalmanFilter实例
        kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            initial_state_mean=[0, 0, 0, 0],
            initial_state_covariance=np.eye(4) * 1000,
            observation_covariance=np.eye(2) * 5,
            transition_covariance=np.eye(4) * 0.5,
            em_vars=['transition_covariance', 'observation_covariance']
        )
        
        return kf
    
    def process(self, measurement):
        """
        处理新的测量值
        
        参数:
            measurement: (x, y)坐标元组，如果检测失败则为None
            
        返回:
            filtered_coords: 滤波后的坐标(x, y)
            status: 滤波器状态 ('collecting', 'initialized', 'predicting')
        """
        current_time = time.time()
        
        # 检查是否需要重置
        if measurement is None:
            if current_time - self.last_detection_time > self.reset_timeout:
                self._reset()
        else:
            self.last_detection_time = current_time
        
        # 如果滤波器未初始化，收集初始样本
        if not self.kalman_initialized:
            if measurement is not None:
                # 如果initial_samples_needed为None，则跳过样本收集和EM训练
                if self.initial_samples_needed is None:
                    # 直接使用当前测量值初始化滤波器
                    self.filtered_state_mean = np.array([measurement[0], 0, measurement[1], 0])
                    self.filtered_state_covariance = np.eye(4) * 10
                    self.kalman_initialized = True
                    return measurement, 'initialized'
                else:
                    self.initial_samples.append(measurement)
                    if len(self.initial_samples) >= self.initial_samples_needed:
                        self._train_kalman_filter()
                        return measurement, 'initialized'
                    return measurement, 'collecting'
            return None, 'collecting'
        
        # 滤波器已初始化，进行滤波或预测
        if measurement is not None:
            # 更新Kalman滤波器
            self.filtered_state_mean, self.filtered_state_covariance = \
                self.kf.filter_update(
                    self.filtered_state_mean,
                    self.filtered_state_covariance,
                    np.array(measurement)
                )
            return (self.filtered_state_mean[0], self.filtered_state_mean[2]), 'initialized'
        else:
            # 预测下一个状态
            self.filtered_state_mean, self.filtered_state_covariance = \
                self.kf.filter_update(
                    self.filtered_state_mean,
                    self.filtered_state_covariance,
                    observation=None
                )
            return (self.filtered_state_mean[0], self.filtered_state_mean[2]), 'predicting'
# ... existing code ...
    
    def _train_kalman_filter(self):
        """使用收集的样本训练Kalman滤波器"""
        # 转换为numpy数组
        samples_array = np.array(self.initial_samples)
        
        # 只有当initial_samples_needed不为None时才使用EM算法估计参数
        if self.initial_samples_needed is not None:
            self.kf = self.kf.em(samples_array, n_iter=10)
        
        # 初始化滤波状态
        last_sample = self.initial_samples[-1]
        self.filtered_state_mean = np.array([last_sample[0], 0, last_sample[1], 0])
        self.filtered_state_covariance = np.eye(4) * 10
        
        self.kalman_initialized = True
        self.initial_samples = []  # 清空样本列表
    
    def _reset(self):
        """重置滤波器"""
        self.kalman_initialized = False
        self.initial_samples = []
        self.filtered_state_mean = None
        self.filtered_state_covariance = None
        self.kf = self._initialize_kalman_filter()
    
    def is_initialized(self):
        """检查滤波器是否已初始化"""
        return self.kalman_initialized
    
    def get_sample_count(self):
        """获取已收集的样本数量"""
        return len(self.initial_samples)