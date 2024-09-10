import rclpy

from .display import Display, Config


def main():
    config = Config(
        threshold=0.7,
        scale=1.0,
    )

    rclpy.init()
    camera = Display(config)
    rclpy.spin(camera)
    rclpy.shutdown()
