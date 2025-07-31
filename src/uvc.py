import os
import time
from threading import Thread

import cv2
from loguru import logger


def time_diff(last_time=[None]):
    """计算两次调用之间的时间差，单位为ns。"""
    current_time = time.time_ns()  # 获取当前时间（单位：ns）

    if last_time[0] is None:  # 如果是第一次调用，更新 last_time
        last_time[0] = current_time
        return 0.000_000_1  # 防止第一次调用时的除零错误

    else:  # 计算时间差
        diff = current_time - last_time[0]  # 计算时间差
        last_time[0] = current_time  # 更新上次调用时间
        return diff  # 返回时间差 ns


class USBCamera:
    """USB摄像头"""

    def __init__(self, config={}):
        self.config = config
        self.img = None
        self.cap = self.init_video_stream()

    def init_video_stream(self, cap_id=None):
        """初始化视频流"""
        if cap_id is None:
            cap_id = self.config.get("camera_id", 0)

        capture = cv2.VideoCapture(cap_id)
        # 设置编码方式
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))  # type: ignore
        # 设置图像高度
        capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT, int(self.config.get("image_width", 1280))
        )
        # 设置图像宽度
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.config.get("image_height", 720)))
        # 设置帧率
        capture.set(cv2.CAP_PROP_FPS, int(self.config.get("fps", 30)))

        capture.set(
            cv2.CAP_PROP_AUTO_EXPOSURE, int(self.config.get("auto_exposure", 1))
        )
        capture.set(cv2.CAP_PROP_EXPOSURE, int(self.config.get("exposure_time", 100)))
        capture.set(cv2.CAP_PROP_GAIN, int(self.config.get("gain", 100)))

        self.cap = capture

        self.cap = capture

        return capture

    def read_color_img(self):
        """获取彩色图"""
        # 通过OpenCV读取彩图
        ret, color_img = self.cap.read()
        return color_img

    def release(self):
        """释放资源"""
        self.cap.release()

    def start(self):
        self.thread = Thread(target=self.loop)
        self.thread.start()
        logger.info("启动捕获线程")

    def stop(self):
        self.thread_stop = True
        self.thread.join()
        logger.info("停止捕获线程")
        self.release()
        logger.info("释放资源")

    def loop(self):

        logger.info("进入 loop 线程")

        cnt = 0
        while not self.cap.isOpened():
            cnt += 1
            print(f"摄像头打开失败，请检查摄像头是否正常连接！{cnt}")
            time.sleep(0.1)
            continue

        while True:

            ret, frame = self.cap.read()

            if ret:
                if os.environ.get("DISPLAY") and os.isatty(0):  # 检查有无图形界面
                    cv2.namedWindow("raw", cv2.WINDOW_NORMAL)
                    cv2.imshow("raw", frame)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.cap.release()
                        cv2.destroyAllWindows()
                        break

                dt = time_diff()

                # print(f"图像大小: {frame.shape}, 帧率 FPS: {1 / (dt/1e9) }, 帧时间: {dt/1e6}")

                self.img = frame  # 缓存图像

            time.sleep(0.001)

        logger.info("结束了 loop 线程")


if __name__ == "__main__":
    camera_params = {
        "camera_id": 0,
        "image_width": 1280,
        "image_height": 720,
        "auto_exposure": 1,
        "exposure_time": 100,
        "fps": 60,
        "gain": 100,
    }

    camera = USBCamera(camera_params)
    camera.start()
