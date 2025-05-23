from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
# import json

# from pydantic import BaseModel
# from queue import Queue
# import time

# from .renderer import Renderer


# class Config(BaseModel):
#     threshold: float
#     scale: float


class Display(Node):
    def __init__(self):
        super().__init__("display")
        # self.create_subscription(
        #     CompressedImage, "/rendered", self.__callback, 1
        # )
        self.create_subscription(CompressedImage, "/rendered", self.__callback, 1)
        self.__cv_bridge = CvBridge()

        cv2.namedWindow("Renderer", cv2.WINDOW_NORMAL)

        self.get_logger().info("Initialized")

    def __callback(self, image: CompressedImage):
        print("Received: /republish")
        cv_image = self.__cv_bridge.compressed_imgmsg_to_cv2(image)

        _, _, width, height = cv2.getWindowImageRect("Renderer")
        print(f"Window size: {width}x{height}")
        # resized_image = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_LINEAR)
        resized_image = cv2.resize(cv_image, dsize=None, fx=0.5,fy=0.5, interpolation = cv2.INTER_LINEAR)

        cv2.imshow("Renderer", resized_image)
        cv2.waitKey(1)

    # def __pose_callback(self, pose: String):
    #     self.get_logger().debug("Received: /pose")
    #     self.__pose_buffer = json.loads(pose.data)

    #     # Calculate FPS
    #     if self.__request_history.full():
    #         self.__request_history.get()

    #     self.__request_history.put(time.time() - self.__last)
    #     self.__last = time.time()

    #     mean = sum(self.__request_history.queue) / len(self.__request_history.queue)
    #     self.__current_fps = 1 / mean
