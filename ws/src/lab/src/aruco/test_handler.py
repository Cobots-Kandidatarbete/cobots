from cv2 import circle
import numpy as np
import threading
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


aruco_stamps = []

def tup_to_int(tup):
    x, y = tup
    return int(x), int(y)


class ArUcoTracker(Node):
    def __init__(self, camera_index=0, marker_size=0.05):
        super().__init__(node_name="aruco_tracker")

        aruco_dict=cv2.aruco.DICT_6X6_50
        cv2.ShowUndistortedImage = True
        self.marker_size = marker_size
        self.capture = cv2.VideoCapture(camera_index)
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # https://docs.opencv.org/3.4/d3/ddc/group__ccalib.html#ga5825788141f468e6fbcd501d5115df15
        self.camera_matrix = np.array([
            [955.9252891407828, 0, 299.2929814576621],
            [0, 958.9317260791769, 193.5121531452791],
            [0, 0, 1]
        ])

        self.distortion_coeffecients = np.array([0, 0, 0, 0], dtype='float')
        xi = np.array([0.0])
        size = (640, 480)
        rectification_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype='float')
        projection_matrix = self.camera_matrix.copy()
        np.append(projection_matrix, np.zeros((2, 1), dtype=float))
        map_1_type = cv2.CV_32F
        flags = cv2.omnidir.RECTIFY_PERSPECTIVE

        self.mapx, self.mapy = cv2.omnidir.initUndistortRectifyMap(
            self.camera_matrix,
            self.distortion_coeffecients,
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
        box_color = 0, 255, 0
        circle_color = 0, 0, 255
        box_thickness = 2
        circle_radius = 4
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_color = 0, 255, 0
        font_scale = 0.5
        font_thickness = 2

        while True:
            
            capture_success, frame = self.capture.read()
            if not capture_success:
                print("Ignoring empty camera frame.")
                continue

            frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
            corneres = np.array(corners)

            if len(corners) > 0:
                ids = ids.flatten()

                for marker_corner, marker_id in zip(corners, ids):
                    top_left, top_right, bot_right, bot_left = marker_corner.reshape((4, 2))
                    top_left = tup_to_int(top_left)
                    top_right = tup_to_int(top_right)
                    bot_right = tup_to_int(bot_right)
                    bot_left = tup_to_int(bot_left)

                    cv2.line(frame, top_left, top_right, box_color, box_thickness)
                    cv2.line(frame, top_right, bot_right, box_color, box_thickness)
                    cv2.line(frame, bot_right, bot_left, box_color, box_thickness)
                    cv2.line(frame, bot_left, top_left, box_color, box_thickness)

                    center = (top_left[0] + bot_right[0]) // 2, (top_left[1] + bot_right[1]) // 2
                    cv2.circle(frame, center, circle_radius, circle_color, -1)

                    cv2.putText(frame, str(marker_id), (top_left[0], top_right[1] - 15), font, font_scale, font_color, font_thickness)

                    # http://amroamroamro.github.io/mexopencv/matlab/cv.estimatePoseSingleMarkers.html
                    rotation_vectors, transformation_vectors, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corner, self.marker_size, self.camera_matrix, self.distortion_coeffecients)
                    rotation = rotation_vectors[0, 0, :]
                    transformation = transformation_vectors[0, 0, :]


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
