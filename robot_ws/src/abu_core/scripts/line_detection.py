import cv2


def process_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, 190, 255, cv2.THRESH_BINARY)
    cv2.imshow(f"Binary Image2", binary)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        M = cv2.moments(largest_contour)
        peri = cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, 0.015 * peri, True)
        if M["m00"] != 0 and area >= 1000:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
            return cx, cy
    return None


cap = cv2.VideoCapture("/dev/video2")

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

while True:
    success, img = cap.read()
    # img = cv2.flip(img, 0)
    # img = cv2.flip(img, 1)
    img = img[int(img.shape[0] / 2) :, :]
    if not success:
        print("Error: Failed to capture image.")
        break

    frame1 = img[:, : int(img.shape[1] / 2) - 80]
    frame2 = img[:, int(img.shape[1] / 2) - 80 : int(img.shape[1] / 2) + 80]
    frame3 = img[:, int(img.shape[1] / 2) + 80 :]

    # # Process the cropped regions
    point1 = process_image(frame1)
    point2 = process_image(frame2)
    point3 = process_image(frame3)

    print(point1, point2, point3)

    # cv2.line(img, (360, 0), (360, 480), (255, 0, 0), 2)
    # cv2.line(img, (440, 0), (440, 480), (255, 0, 0), 2)
    cv2.imshow("Image", img)
    cv2.imshow("Binary Image", frame3)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
