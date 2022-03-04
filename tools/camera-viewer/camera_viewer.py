import cv2
from cv2 import VideoCapture
import time
import numpy as np


def main():
    index = 0
    new_index = index
    capture = cv2.VideoCapture(index)
    while capture.isOpened():
        ret, frame = capture.read()
        if ret:
            cv2.imshow("Frame", frame)

            key_input = cv2.waitKey(25) & 0xFF

            if key_input == ord('q'):
                break


            if ord('0') <= key_input <= ord('9') :
                new_index = key_input - ord('0')


            if new_index != index:
                capture.release()
                new_camera = cv2.VideoCapture(new_index)
                time.sleep(0.5)
                if new_camera.isOpened():
                    capture = new_camera
                    index = new_index

                else:
                    capture = VideoCapture(index)
                    new_index = index


        else:
            break
    print("RELEASED CAMERA")

    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()