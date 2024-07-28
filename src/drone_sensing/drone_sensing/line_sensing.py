import rclpy
from rclpy.node import Node
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LineSensing(Node):

    def __init__(self):
        super().__init__('line_sensing')
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Listening to /image_raw topic")

    def get_largest_contour(self, contours, min_area=100):
        if len(contours) == 0:
            return None
        greatest_area = min_area
        greatest_contour = None
        for contour in contours:
            if cv.contourArea(contour) > greatest_area:
                greatest_contour = contour
                greatest_area = cv.contourArea(contour)
        return greatest_contour

    def split_image_by_rows(self, image, n, start_from_middle=True):
        if n <= 0:
            raise ValueError("The number of parts `n` must be greater than 0.")
        
        height, width = image.shape[:2]
        part_height = height // (2 * n)  # Splitting only the upper half
        parts = []

        if start_from_middle:
            for i in range(n):
                start_row = height // 2 - (i + 1) * part_height
                end_row = height // 2 - i * part_height
                part = image[start_row:end_row, :]
                parts.append((part, start_row))  # Include the starting row index
        
        return parts

    def draw_center_line(self, image, dot_length, gap_length, thickness, color):
        height, width = image.shape[:2]
        
        for x in range(0, width, dot_length + gap_length):
            cv.line(image, (x, height // 2), (x + dot_length, height // 2), color, thickness)
        for y in range(0, height, dot_length + gap_length):
            cv.line(image, (width // 2, y), (width // 2, y + dot_length), color, thickness)

    def get_direction(self, image, cx):
        center = image.shape[1] / 2
        if cx is None:
            return "NO LINE DETECTED"
        elif cx < center:
            return "MOVE RIGHT"
        elif cx > center:
            return "MOVE LEFT"
        else:
            return "CENTERED"

    def contour_and_centroid(self, image, n):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
        parts = self.split_image_by_rows(image, n)  # Change number of segments
        
        centroid_x, centroid_y = None, None
        
        for part, start_row in parts:  # Iterate from middle upwards
            contours = cv.findContours(thresh[start_row:start_row+part.shape[0], :], cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]
            largest_contour = self.get_largest_contour(contours)
            if largest_contour is not None:
                M = cv.moments(largest_contour)
                if M["m00"] != 0:
                    centroid_x = int(M["m10"] / M["m00"])
                    centroid_y = int(M["m01"] / M["m00"]) + start_row  # Adjust for the position in the whole image
                    cv.circle(image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
                    cv.drawContours(image, [largest_contour + [0, start_row]], 0, (0, 255, 0), 2)  # Adjust contour position
                    break  # Stop after processing the first part with contours

        self.get_logger().info(self.get_direction(image, centroid_x))
        self.draw_center_line(image, 10, 10, 1, (255, 0, 0)) 
        
        return image, centroid_x, centroid_y

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        # Process the image
        processed_frame, cx, cy = self.contour_and_centroid(frame, 1)
        
        # Display the processed frame
        cv.imshow('Processed Frame', processed_frame)
        cv.waitKey(1)  # Add a small delay to keep the window responsive

def main(args=None):
    rclpy.init(args=args)
    node = LineSensing()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
