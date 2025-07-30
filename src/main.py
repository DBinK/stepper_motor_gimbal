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
    
    while True:
        img = cam.read_color_img() 
        if img is None:
            continue
        vertices, intersection = detector.detect(img)
        if vertices is not None:
            img = detector.draw(img, vertices, intersection)
        cv2.imshow("img", img)
        if cv2.waitKey(1) == ord("q"):
            break
    pass
