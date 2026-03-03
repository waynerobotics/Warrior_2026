import cv2

def detect_cameras(max_devices=5):
    print("Searching for available cameras...")
    for i in range(max_devices):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"✅ Camera detected at index {i}")
            cap.release()
        else:
            print(f"❌ No camera at index {i}")

detect_cameras()

