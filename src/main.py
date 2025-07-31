import cv2
import time

from detector import Detector
from uvc import USBCamera
from gimbal import Gimbal
from filters import CoordinateKalmanFilter  # 导入新的滤波器类

from utils import time_diff

# 初始化检测器
detector = Detector(
    max_perimeter=99999,
    min_perimeter=1,
    min_angle=30,
)

# 初始化相机
# cam = USBCamera({'camera_id': 1,})
cam = USBCamera(
    {
        "camera_id": "http://192.168.0.110:4747/video",
    }
)

# 初始化云台
# gim = Gimbal(horizontal_port='COM28', vertical_port='COM28')

# 初始化坐标滤波器
coord_filter = CoordinateKalmanFilter(
    fps=30,
    initial_samples_needed=None,
    reset_timeout=1.0,
    jump_threshold=500,  # 设置跳变阈值
)

if __name__ == "__main__":
    while True:
        # 获取图像
        img = cam.read_color_img()
        if img is None:
            continue

        # 检测靶纸
        vertices, intersection = detector.detect(img)

        # 处理坐标 - 核心变化在这里
        measurement = intersection  # 可以是None
        filtered_coords, status = coord_filter.process(measurement)

        # 绘制检测结果
        img = detector.draw(img, vertices, intersection)

        # 绘制滤波后的点
        if filtered_coords is not None:
            x, y = int(filtered_coords[0]), int(filtered_coords[1])
            if status == "initialized":
                cv2.circle(img, (x, y), 5, (0, 255, 0), -1)  # 绿色: 滤波后
            elif status == "predicting":
                cv2.circle(img, (x, y), 5, (255, 0, 0), -1)  # 蓝色: 预测

            # 云台控制 (无云台时可注释后调试)
            # if status in ['initialized', 'predicting']:
            #     gim.auto_track_target(x, y, img.shape[1], img.shape[0])

        # 显示状态信息
        if not coord_filter.is_initialized():
            status_text = f"Collecting samples: {coord_filter.get_sample_count()}/{coord_filter.initial_samples_needed}"
        elif measurement is None:
            status_text = "Tracking (predicti mode)"
        else:
            status_text = "Tracking (filter mode)"

        cv2.putText(
            img,
            f"Kalman: {status_text}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        if filtered_coords is not None and coord_filter.is_initialized():
            cv2.putText(
                img,
                f"Filtered: ({int(filtered_coords[0])}, {int(filtered_coords[1])})",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        # 显示图像
        cv2.namedWindow("Kalman Filtered Tracking", cv2.WINDOW_NORMAL)
        cv2.imshow("Kalman Filtered Tracking", img)

        if cv2.waitKey(1) == ord("q"):
            break

        # dt = time_diff()
        # print(f"图像大小: {img.shape}, 帧率 FPS: {1 / (dt/1e9) }, 帧时间: {dt/1e6}")

    cv2.destroyAllWindows()
