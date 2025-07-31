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
gim = Gimbal(horizontal_port='COM28', vertical_port='COM28')


if __name__ == "__main__":

    while True:
        # 获取图像
        img = cam.read_color_img() 
        if img is None:
            continue

        # 检测靶纸
        vertices, intersection = detector.detect(img)  
        print(f"中点坐标: {intersection}")

        # 绘制检测结果
        if vertices is not None: 
            img = detector.draw(img, vertices, intersection)

        # 云台控制 (无云台时可注释后调试)
        if intersection is not None: 
            gim.auto_track_target(intersection[0], intersection[1], img.shape[1], img.shape[0])
        
        # 显示图像
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)

        if cv2.waitKey(1) == ord("q"):
            break
    
