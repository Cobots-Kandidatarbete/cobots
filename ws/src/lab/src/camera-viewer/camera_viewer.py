import cv2
import time


def main():

    index = 0
    new_index = index
    capture = cv2.VideoCapture(index)

    print("Press 'q' to exit program.")
    print("Press keys 0-9 to preview camera at given index.")

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
                new_capture = cv2.VideoCapture(new_index)
                time.sleep(0.5)

                if new_capture.isOpened():
                    capture = new_capture
                    index = new_index

                else:
                    new_index = index


        else:
            break
    
    print("Exiting program")

    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()