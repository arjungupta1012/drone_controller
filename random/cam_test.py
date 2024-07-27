import cv2

def test_camera():
    # Open the camera
    cap = cv2.VideoCapture(0)  # 0 corresponds to the default camera, adjust if necessary

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If frame is read correctly, ret is True
        if not ret:
            print("Error: Cannot receive frame (stream end?). Exiting...")
            break

        # Display the frame
        cv2.imshow('Camera Test', frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera()
