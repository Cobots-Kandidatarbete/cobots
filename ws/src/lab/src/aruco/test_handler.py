import numpy as np
import threading
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


aruco_stamps = []


class ArUcoTracker(Node):
    def __init__(self, camera_index=0):
        super().__init__(node_name="aruco_tracker")
        aruco_dict=cv2.aruco.DICT_6X6_50
        cv2.ShowUndistortedImage = True
        self.capture = cv2.VideoCapture(camera_index)
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # https://docs.opencv.org/3.4/d3/ddc/group__ccalib.html#ga5825788141f468e6fbcd501d5115df15
        camera_matrix = np.array([
            [955.9252891407828, 0, 299.2929814576621],
            [0, 958.9317260791769, 193.5121531452791],
            [0, 0, 1]
        ])

        distortion_coeffecients = np.array([0, 0, 0, 0], dtype='float')
        xi = np.array([0.0])
        size = (640, 480)
        rectification_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype='float')
        projection_matrix = camera_matrix.copy()
        np.append(projection_matrix, np.zeros((2, 1), dtype=float))
        map_1_type = cv2.CV_32F
        flags = cv2.omnidir.RECTIFY_PERSPECTIVE

        self.mapx, self.mapy = cv2.omnidir.initUndistortRectifyMap(
            camera_matrix,
            distortion_coeffecients,
            xi,
            rectification_matrix,
            projection_matrix,
            size,
            map_1_type,
            flags
        )

        self.get_logger().info("aruco_tracker node should be started")
        t1 = threading.Thread(target=self.run_vision_callback)
        t1.daemon = True
        t1.start()
        
    def run_vision_callback(self):
        while True:
            print("Hello")


class ArUcoPublisher(Node):
    pass


if __name__ == "__main__":
    rclpy.init()
    try:
        c1 = ArUcoTracker()
        #c2 = ArUcoPublisher()
        
        executor = MultiThreadedExecutor()
        executor.add_node(c1)
        #executor.add_node(c2)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            c1.destroy_node()
#            c2.()            

    finally:
        rclpy.shutdown()
