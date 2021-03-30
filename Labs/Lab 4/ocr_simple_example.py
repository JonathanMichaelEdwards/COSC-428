# ocr_simple_example.py

import pytesseract as tess
import cv2

image = cv2.imread("simple_text.png", cv2.IMREAD_GRAYSCALE)
text = tess.image_to_string(image)
print(text)
