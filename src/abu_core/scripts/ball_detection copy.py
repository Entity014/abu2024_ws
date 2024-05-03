# finding hsv range of target object(pen)
import cv2
import numpy as np
import time


# A required callback method that goes into the trackbar function.
def nothing(x):
    pass


# Initializing the webcam feed.
def find_HSV(source, mode: str):
    mode = mode.upper()
    if mode == "CAM" or mode == "VIDEO":
        cap = cv2.VideoCapture(source)
        cap.set(3, 1280)
        cap.set(4, 720)
    elif mode == "IMG":
        frame = cv2.imread(source)

    # Create a window named trackbars.
    cv2.namedWindow("Trackbars")

    # Now create 6 trackbars that will control the lower and upper range of
    # H,S and V channels. The Arguments are like this: Name of trackbar,
    # window name, range,callback function. For Hue the range is 0-179 and
    # for S,V its 0-255.
    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

    while True:
        # Start reading the webcam feed frame by frame.
        if mode == "CAM" or mode == "VIDEO":
            ret, frame = cap.read()
            if not ret:
                break

        # Convert the BGR image to HSV image.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get the new values of the trackbar in real time as the user changes
        # them
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        # Set the lower and upper HSV range according to the value selected
        # by the trackbar
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])

        # Filter the image and get the binary mask, where white represents
        # your target color
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # You can also visualize the real part of the target color (Optional)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Converting the binary mask to 3 channel image, this is just so
        # we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # stack the mask, orginal frame and the filtered result
        stacked = np.hstack((mask_3, frame, res))

        # Show this stacked frame at 40% of the size.
        cv2.imshow("Trackbars", cv2.resize(stacked, None, fx=0.4, fy=0.4))

        # If the user presses ESC then exit the program
        key = cv2.waitKey(30)
        if key == 27:
            break

        # If the user presses `s` then print this array.
        if key == ord("s"):
            thearray = [[l_h, l_s, l_v], [u_h, u_s, u_v]]
            print(thearray)

            # Also save this array as penval.npy
            np.save("hsv_value", thearray)
            break
        if mode == "CAM" or mode == "VIDEO":
            if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    # Release the camera & destroy the windows.
    if mode == "CAM" or mode == "VIDEO":
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # ? ESC to exit the program
    # ? S to save hsv.npy
    # ? Mode is [ CAM, IMG, VIDEO ]
    # ? CAM mode source only int
    find_HSV(
        source="/dev/video14",
        mode="video",
    )