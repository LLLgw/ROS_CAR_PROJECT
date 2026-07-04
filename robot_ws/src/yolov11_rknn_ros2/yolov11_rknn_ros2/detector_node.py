import os
import time

import cv2
import rclpy
from rclpy.node import Node

from .func.func_yolov8_optimize import CLASSES
from .rknnpool.rknnpool_ld import rknnPoolExecutor


class YoloV11RknnDetectorNode(Node):
    def __init__(self):
        super().__init__("yolov11_rknn_detector")

        # ROS 2 parameters around the original Python demo workflow.
        self.declare_parameter("model_path", "")
        self.declare_parameter("video_device", "/dev/video21")
        self.declare_parameter("tpes", 8)
        self.declare_parameter("enable_display", True)
        self.declare_parameter("display_width", 1420)
        self.declare_parameter("display_height", 800)
        self.declare_parameter("fullscreen", False)
        self.declare_parameter("flip_code", 1)

        model_path = self.get_parameter("model_path").value
        video_device = self.get_parameter("video_device").value
        self.tpes = int(self.get_parameter("tpes").value)
        self.enable_display = bool(self.get_parameter("enable_display").value)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.flip_code = int(self.get_parameter("flip_code").value)

        if not model_path or not os.path.exists(model_path):
            raise FileNotFoundError(f"RKNN model not found: {model_path}")

        # OpenCV reads frames directly from the device file, matching the original demo behavior.
        self.cap = cv2.VideoCapture(video_device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open video device: {video_device}")

        # Reuse the original multi-thread RKNN pipeline instead of rewriting inference logic.
        self.pool = rknnPoolExecutor(
            rknnModel=model_path,
            TPEs=self.tpes,
            func=self.infer_frame,
        )

        if self.enable_display:
            cv2.namedWindow("yolov11_rknn_result", cv2.WINDOW_NORMAL)
            if self.fullscreen:
                cv2.setWindowProperty(
                    "yolov11_rknn_result",
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )

        # Fill the original RKNN pipeline before entering the main loop.
        self.prime_pipeline()

        self.frames = 0
        self.loop_time = time.time()
        self.init_time = time.time()

        self.get_logger().info(f"Opened video device: {video_device}")
        self.get_logger().info(f"Loaded RKNN model: {model_path}")
        self.get_logger().info(f"Loaded class labels: {', '.join(CLASSES)}")
        self.get_logger().info(f"Realtime mode enabled with TPEs={self.tpes}")
        self.get_logger().info(f"flip_code={self.flip_code}, local display only")

    def prime_pipeline(self):
        for _ in range(self.tpes + 1):
            ret, frame = self.cap.read()
            if not ret:
                raise RuntimeError("Failed to read frame while priming the RKNN pipeline")
            frame = self.maybe_flip(frame)
            self.pool.put(frame)

    def run(self):
        # Keep the execution path as close as possible to the original script.
        while rclpy.ok() and self.cap.isOpened():
            self.frames += 1
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("Failed to read frame from video device")
                break

            frame = self.maybe_flip(frame)
            self.pool.put(frame)
            annotated, ok = self.pool.get()
            if not ok:
                self.get_logger().warning("Failed to get inference result from RKNN pool")
                break

            if self.frames % 30 == 0:
                now = time.time()
                fps = 30 / (now - self.loop_time)
                self.get_logger().info(f"average fps over 30 frames={fps:.2f}")
                self.loop_time = now

            if self.enable_display:
                display_frame = cv2.resize(annotated, (self.display_width, self.display_height))
                cv2.imshow("yolov11_rknn_result", display_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        total_fps = self.frames / max(time.time() - self.init_time, 1e-6)
        self.get_logger().info(f"overall average fps={total_fps:.2f}")

    def maybe_flip(self, frame):
        if self.flip_code in (-1, 0, 1):
            return cv2.flip(frame, self.flip_code)
        return frame

    def infer_frame(self, rknn_lite, frame):
        # Import locally so the node stays a thin ROS 2 wrapper around the original demo code.
        from .func.func_yolov8_optimize import myFunc

        return myFunc(rknn_lite, frame)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        if hasattr(self, "pool"):
            self.pool.release()
        if self.enable_display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloV11RknnDetectorNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
