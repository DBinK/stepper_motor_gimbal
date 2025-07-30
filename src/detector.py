
import cv2
import numpy as np
import sys
from loguru import logger

# 设置日志级别 
logger.remove()
# logger.add(sys.stderr, level="DEBUG") 
logger.add(sys.stderr, level="INFO")  


class QuadDetector:
    """
    四边形检测类
    """
    def __init__(self, max_perimeter=99999, min_perimeter=1, min_angle=30):
        """
        @param img: 图像来源
        @param max_perimeter: 允许的最大周长
        @param min_perimeter: 允许的最小周长
        @param scale: 缩放比例
        @param min_angle: 允许的最小角度
        @param line_seg_num: 分段数量
        """
        self.img = None

        self.max_perimeter = max_perimeter
        self.min_perimeter = min_perimeter
        self.min_angle     = min_angle

    def preprocess_image(self, img: np.ndarray):

        """
        对输入图像进行预处理, 包括灰度转换、高斯模糊、Canny边缘检测, 并返回其中的轮廓信息。
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像

        blur = cv2.GaussianBlur(gray, (1, 1), 0)  # 高斯滤波去噪

        # 减小曝光
        # exposure_adjusted = cv2.addWeighted(blur, 0.5, np.zeros(blur.shape, dtype=blur.dtype), 0, 50)

        # 增加对比度（直方图均衡化）
        # blur = cv2.convertScaleAbs(blur, alpha=0.5, beta=-50)
        # blur = cv2.convertScaleAbs(blur, alpha=1, beta=-120)
        
        # 二值化
        # _, blur = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # _, threshold = cv2.threshold(blur, 157, 255, cv2.THRESH_BINARY) # 二值化

        edges = cv2.Canny(blur, 50, 200)

        return edges


    def find_max_quad_vertices(self, pre_img):
        """
        在预处理后的图像中寻找具有最大周长的四边形，并返回顶点坐标
        """
        contours, _ = cv2.findContours(pre_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # 查找轮廓

        logger.debug(f'contours cnt: {len(contours)}')

        max_perimeter_now = 0
        vertices = None

        # 遍历轮廓列表
        for cnt in contours:
            # 将当前轮廓近似为四边形
            approx = cv2.approxPolyDP(cnt, 0.09 * cv2.arcLength(cnt, True), True)

            # 确保转换后的形状为四边形
            if len(approx) == 4:
                # 计算四边形周长
                perimeter = cv2.arcLength(approx, True)
                perimeter_allowed = (perimeter <= self.max_perimeter) and (perimeter >= self.min_perimeter)
                # cv2.drawContours(img, [approx], 0, (255, 0, 0), 2)

                if perimeter_allowed and perimeter > max_perimeter_now:
                    # 计算四边形角度
                    cosines = []
                    for i in range(4):
                        p0 = approx[i][0]
                        p1 = approx[(i + 1) % 4][0]
                        p2 = approx[(i + 2) % 4][0]
                        v1 = p0 - p1
                        v2 = p2 - p1
                        cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                        angle = np.arccos(cosine_angle) * 180 / np.pi
                        cosines.append(angle)
                        
                    # 若当前轮廓周长在允许范围内、大于当前最大周长且角度大于 min_angle
                    if all(angle >= self.min_angle for angle in cosines):
                        logger.info(f"perimeter: {perimeter}")
                        max_perimeter_now = perimeter
                        vertices = approx.reshape(4, 2)
                    else:
                        vertices = None

        if vertices is not None:
            logger.info(f"Found vertices: {vertices.tolist()}")
        
        return vertices
    

    def calculate_intersection(self, vertices):
        """
        @description: 计算四边形对角线的交点。
        @param vertices: 输入的顶点坐标
        @return: 返回交点坐标
        """

        x1, y1 = vertices[0]
        x2, y2 = vertices[2]
        x3, y3 = vertices[1]
        x4, y4 = vertices[3]

        dx1, dy1 = x2 - x1, y2 - y1
        dx2, dy2 = x4 - x3, y4 - y3

        det = dx1 * dy2 - dx2 * dy1

        if det == 0 or (dx1 == 0 and dx2 == 0) or (dy1 == 0 and dy2 == 0):
            return None

        dx3, dy3 = x1 - x3, y1 - y3
        det1 = dx1 * dy3 - dx3 * dy1
        det2 = dx2 * dy3 - dx3 * dy2

        if det1 == 0 or det2 == 0:
            return None

        s = det1 / det
        t = det2 / det

        if 0 <= s <= 1 and 0 <= t <= 1:
            intersection_x = int(x1 + dx1 * t)
            intersection_y = int(y1 + dy1 * t)
            self.intersection = [intersection_x, intersection_y]

            logger.info(f"Found intersection: {self.intersection}")

            return intersection_x, intersection_y
        else:
            logger.info("No intersection found.")
            return []
        
    def detect(self,img):
        """
        @description: 检测函数入口
        """

        pre_img = self.preprocess_image(img.copy())

        vertices       = self.find_max_quad_vertices(pre_img)
        if vertices is None:
            raise ValueError("No quadrilateral found.")
        intersection   = self.calculate_intersection(vertices)

        return vertices, intersection
    
    def draw(self, img,vertices, intersection):
        """
        @description: 绘制检测结果
        """
        def draw_point_text(img, x, y, bgr = ( 0, 0, 255)): #绘制一个点，并显示其坐标。
            cv2.circle(img, (x, y), 6, bgr, -1)
            cv2.putText(
                img,
                f"({x}, {y})",
                (x + 5, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 1, cv2.LINE_AA,
            )
            return img

        def draw_lines_points(img, vertices, bold=2):
            # 绘制轮廓
            cv2.drawContours(img, [vertices], 0, (255, 0, 0), bold)

            for _, vertex in enumerate(vertices):  # 绘制每个角点和坐标
                draw_point_text(img, vertex[0], vertex[1])
            
            cv2.line(  # 绘制对角线
                img,
                (vertices[0][0], vertices[0][1]),
                (vertices[2][0], vertices[2][1]),
                (0, 255, 0), 1,
            )
            cv2.line(
                img,
                (vertices[1][0], vertices[1][1]),
                (vertices[3][0], vertices[3][1]),
                (0, 255, 0), 1,
            )
            return img
    

        img_drawed = draw_lines_points(img, vertices)          # 绘制最大四边形
        img_drawed = draw_point_text(img_drawed, intersection[0], intersection[1]) # 绘制交点

        return img_drawed


if __name__ == '__main__':

    print("开始测试")
    
    img = cv2.imread("img/1.jpg")
    if img is None:
        logger.error("图像加载失败，请检查路径是否正确或图像格式是否支持。")
        exit(-1)
    
    # 初始化四边形检测器
    quad_detector = QuadDetector()

    quad_detector.max_perimeter = 99999
    quad_detector.min_perimeter = 1
    quad_detector.min_angle     = 30

    # 四边形检测结果
    vertices, intersection = quad_detector.detect(img)
    img_detected = quad_detector.draw(img, vertices, intersection)  # 绘制检测结果

    # 显示结果
    cv2.imshow("img_src", img)
    cv2.imshow("img_detected", img_detected)
    cv2.imwrite("img/detected.jpg", img_detected)
    cv2.waitKey(0)