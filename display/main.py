import rclpy

from .display import Display


def main():
    rclpy.init()

    camera = Display()

    rclpy.spin(camera)
    rclpy.shutdown()
