import cv2
import numpy as np

cap = cv2.VideoCapture(-1)  # Open the first camera connected to the computer.

# Callbacks for the trackbar.
def updateBrightness(brightness):
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness/100)

def updateContrast(contrast):
    cap.set(cv2.CAP_PROP_CONTRAST, contrast/100)

def updateSaturation(saturation):
    cap.set(cv2.CAP_PROP_SATURATION, saturation/100)

# Create a window and add two trackbars for controlling the thresholds.
cv2.namedWindow('Camera Settings')
cv2.createTrackbar('Brightness', 'Camera Settings', 0, 100, updateBrightness)
cv2.createTrackbar('Contrast', 'Camera Settings', 0, 100, updateContrast)
cv2.createTrackbar('Saturation', 'Camera Settings', 0, 100, updateSaturation)


while True:
    # Update the image using the latest threshold.
    ret, frame = cap.read()
    cv2.imshow('Camera Settings', frame)

    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
