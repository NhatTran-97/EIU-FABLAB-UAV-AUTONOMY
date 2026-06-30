import cv2
import os

# Function to generate GStreamer pipeline
def gstreamer_pipeline(sensor_id=0, capture_width=640, capture_height=480, framerate=30, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={capture_width}, height={capture_height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink"
    )

# Open both cameras
cap0 = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0), cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(gstreamer_pipeline(sensor_id=1), cv2.CAP_GSTREAMER)

if not cap0.isOpened() or not cap1.isOpened():
    print("Error: One or both cameras not opened")
    exit()

while True:
    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()

    if not ret0 or not ret1:
        print("Failed to capture image from one or both cameras")
        break

    # Display the frames side by side
    combined_frame = cv2.hconcat([frame0, frame1])
    cv2.imshow("Dual Camera Feed", combined_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap0.release()
cap1.release()
cv2.destroyAllWindows()