import rclpy
import os

from .display import Display, Config


def main():
    config = Config(
        threshold=float(os.environ["POSE_THRESHOLD"]),
        scale=2.0,
    )

    rclpy.init()
    camera = Display(config)
    rclpy.spin(camera)
    rclpy.shutdown()
