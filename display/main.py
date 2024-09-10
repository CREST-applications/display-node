import rclpy

from .display import Display, Config


def main():
    config = Config(
        camera_topic="/image_raw/compressed",
        pose_topic="/pose",
        threshold=0.3,
        scale=1.0,
    )

    rclpy.init()
    camera = Display(config)
    rclpy.spin(camera)
    rclpy.shutdown()
