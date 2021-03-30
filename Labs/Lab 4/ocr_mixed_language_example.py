# ocr_mixed_language_example.py

import pytesseract as tess
import cv2

image = cv2.imread("mixed_language_text.png", cv2.IMREAD_GRAYSCALE)
text = tess.image_to_string(image, lang="jpn+eng")
print(text)
