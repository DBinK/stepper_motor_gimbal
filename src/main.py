import cv2

from detector import Detector
from uvc import USBCamera

detector = Detector(
    max_perimeter=99999,
    min_perimeter=1,
    min_angle=30,
)

cam = USBCamera()

cam.start()

if __name__ == "__main__":
    pass
