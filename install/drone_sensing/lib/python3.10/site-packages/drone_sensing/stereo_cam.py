#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import math
import time

class HostSpatialsCalc:
    def __init__(self, device):
        self.calibData = device.readCalibration()
        self.DELTA = 5
        self.THRESH_LOW = 200
        self.THRESH_HIGH = 30000

    def setLowerThreshold(self, threshold_low):
        self.THRESH_LOW = threshold_low

    def setUpperThreshold(self, threshold_high):
        self.THRESH_HIGH = threshold_high

    def setDeltaRoi(self, delta):
        self.DELTA = delta

    def _check_input(self, roi, frame):
        if len(roi) == 4: return roi
        if len(roi) != 2: raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
        self.DELTA = 5
        x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
        y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
        return (x-self.DELTA, y-self.DELTA, x+self.DELTA, y+self.DELTA)

    def _calc_angle(self, frame, offset, HFOV):
        return math.atan(math.tan(HFOV / 2.0) * offset / (frame.shape[1] / 2.0))

    def calc_spatials(self, depthData, roi, averaging_method=np.mean):
        depthFrame = depthData.getFrame()
        roi = self._check_input(roi, depthFrame)
        xmin, ymin, xmax, ymax = roi
        depthROI = depthFrame[ymin:ymax, xmin:xmax]
        inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)
        HFOV = np.deg2rad(self.calibData.getFov(dai.CameraBoardSocket(depthData.getInstanceNum()), useSpec=False))
        averageDepth = averaging_method(depthROI[inRange])
        centroid = {
            'x': int((xmax + xmin) / 2),
            'y': int((ymax + ymin) / 2)
        }
        midW = int(depthFrame.shape[1] / 2)
        midH = int(depthFrame.shape[0] / 2)
        bb_x_pos = centroid['x'] - midW
        bb_y_pos = centroid['y'] - midH
        angle_x = self._calc_angle(depthFrame, bb_x_pos, HFOV)
        angle_y = self._calc_angle(depthFrame, bb_y_pos, HFOV)
        spatials = {
            'z': averageDepth,
            'x': averageDepth * math.tan(angle_x),
            'y': -averageDepth * math.tan(angle_y)
        }
        return spatials, centroid

class TextHelper:
    def __init__(self) -> None:
        self.bg_color = (0, 0, 0)
        self.color = (255, 255, 255)
        self.text_type = cv2.FONT_HERSHEY_SIMPLEX
        self.line_type = cv2.LINE_AA

    def putText(self, frame, text, coords):
        cv2.putText(frame, text, coords, self.text_type, 0.5, self.bg_color, 3, self.line_type)
        cv2.putText(frame, text, coords, self.text_type, 0.5, self.color, 1, self.line_type)

    def rectangle(self, frame, p1, p2):
        cv2.rectangle(frame, p1, p2, self.bg_color, 3)
        cv2.rectangle(frame, p1, p2, self.color, 1)

class FPSHandler:
    def __init__(self):
        self.timestamp = time.time() + 1
        self.start = time.time()
        self.frame_cnt = 0

    def next_iter(self):
        self.timestamp = time.time()
        self.frame_cnt += 1

    def fps(self):
        return self.frame_cnt / (self.timestamp - self.start)

class StereoCam(Node):

    def __init__(self):
        super().__init__('stereo_cam')
        self.stereo_cam_pub = self.create_publisher( #TODO, #TODO, 10))  # message type, topic name, queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.run_depthai) #TODO update name

    def run_depthai(self):
        pipeline = dai.Pipeline()
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo.initialConfig.setConfidenceThreshold(255)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        stereo.depth.link(xoutDepth.input)
        xoutDisp = pipeline.create(dai.node.XLinkOut)
        xoutDisp.setStreamName("disp")
        stereo.disparity.link(xoutDisp.input)

        with dai.Device(pipeline) as device:
            depthQueue = device.getOutputQueue(name="depth")
            dispQ = device.getOutputQueue(name="disp")
            text = TextHelper()
            fpsHandler = FPSHandler()
            hostSpatials = HostSpatialsCalc(device)
            y = 200
            x = 300
            step = 3
            delta = 5
            hostSpatials.setDeltaRoi(delta)
            print("Use WASD keys to move ROI.\nUse 'r' and 'f' to change ROI size.")
            while True:
                depthData = depthQueue.get()
                spatials, centroid = hostSpatials.calc_spatials(depthData, (x, y))
                distance = spatials['z'] / 1000
                print(f"Distance: {distance:.2f}m")  # Print distance for debugging
                disp = dispQ.get().getFrame()
                disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
                disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)
                text.rectangle(disp, (x - delta, y - delta), (x + delta, y + delta))
                text.putText(disp, f"Distance: {distance:.2f}m", (x + 10, y + 20))
                cv2.imshow("depth", disp)
                fpsHandler.next_iter()
                fps = fpsHandler.fps()
                text.putText(disp, f"FPS: {fps:.2f}", (10, 30))
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break
                elif key == ord('w'):
                    y -= step
                elif key == ord('a'):
                    x -= step
                elif key == ord('s'):
                    y += step
                elif key == ord('d'):
                    x += step
                elif key == ord('r'):
                    if delta < 50:
                        delta += 1
                        hostSpatials.setDeltaRoi(delta)
                elif key == ord('f'):
                    if 3 < delta:
                        delta -= 1
                        hostSpatials.setDeltaRoi(delta)

def main(args=None):
    rclpy.init(args=args)
    node = StereoCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
