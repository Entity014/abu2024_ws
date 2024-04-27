import cv2
import numpy as np

cap = cv2.VideoCapture("/dev/video14")


def search_contours(contours):
    global cx, cy
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        epsilon = 0.001 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if area > 3000 and len(approx) >= 10:
            largest_contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.circle(
                frame, (cx, cy), 5, (0, 0, 255), -1
            )  # Mark center with a red circle
            cv2.putText(
                frame,
                f"{len(approx)}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)


r_low = np.array([120, 25, 25])
r_up = np.array([175, 255, 175])
b_low = np.array([85, 60, 45])
b_up = np.array([120, 255, 255])

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    blur_image = cv2.GaussianBlur(frame, (15, 15), 0)
    hsv_frame = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
    # r_mask = cv2.inRange(hsv_frame, r_low, r_up)
    b_mask = cv2.inRange(hsv_frame, b_low, b_up)
    # r_contours, _ = cv2.findContours(r_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    b_contours, _ = cv2.findContours(b_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # search_contours(r_contours)
    search_contours(b_contours)
    cv2.line(frame, (230, 0), (230, 480), (255, 0, 0), 2)
    cv2.line(frame, (280, 0), (280, 480), (255, 0, 0), 2)
    # cv2.imshow("Marked Centers1", res_r_frame)
    # cv2.imshow("Marked Centers2", res_b_frame)

    # Display the captured frame
    cv2.imshow("Camera", frame)

    # Check for 'q' key pressed to exit the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the camera and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
