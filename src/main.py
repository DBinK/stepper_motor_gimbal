import cv2

from detector import Detector
from uvc import USBCamera

from gimbal import Gimbal

# 初始化检测器
detector = Detector(
    max_perimeter=99999,
    min_perimeter=1,
    min_angle=30,
)

# 初始化相机
cam = USBCamera()

# 初始化云台
# gim = Gimbal()

if __name__ == "__main__":

    while True:
        img = cam.read_color_img() 
        if img is None:
            continue
        vertices, intersection = detector.detect(img)
        if vertices is not None:
            img = detector.draw(img, vertices, intersection)

        print(f"中点坐标: {intersection}")

        # 云台控制 (未写)
        
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord("q"):
            break
    pass
