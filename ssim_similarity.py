import pytesseract
from PIL import Image
import matplotlib.pyplot as plt
import cv2
import numpy as np
from skimage import measure
from skimage.metrics import structural_similarity
from reportlab.pdfgen import canvas

weight = [1, 1, 1] # weight for OCR , MSE, SSIM, Hamming Distance

input_text = 'a' # the word the children wanna learn
image_pathC = '/home/shen/letter/NA-Redback/cat-ao.jpg' # children writing
image_pathR = '/home/shen/letter/NA-Redback/cat-a.jpg' # robot writing
# Read images
child_image = cv2.imread(image_pathC, cv2.IMREAD_GRAYSCALE)
robot_image = cv2.imread(image_pathR, cv2.IMREAD_GRAYSCALE)

# OCR
# pytesseract.pytesseract.tesseract_cmd = r'/home/shen/letter/NA-Redback/tesseract'
# Load the image with handwritten text
img = Image.open(image_pathC)
# Perform OCR using Tesseract
custom_config = r'--oem 3 --psm 6'
text = pytesseract.image_to_string(img,lang='eng', config=custom_config)
text.replace(' ', '')
print(text)
total_characters = len(text)
correct_characters = sum(1 for i, j in zip(text, input_text) if i == j)
ocr = (correct_characters / total_characters)
# Print the result
print(f"Letter Binary: {ocr:.2f}")

# Ensure image has same shape
if child_image.shape != robot_image.shape:
    raise ValueError("Shape of two images different")
# MSE
mse = cv2.mean((child_image - robot_image) ** 2)[0]/60
print(f"MSE: {mse}")
# Structural Similarity Index，SSIM
(score, diff) = structural_similarity(child_image, robot_image, win_size=101, full=True)
ssim = score
print("SSIM:{}".format(score))
# Hamming Distance
hamming_distance = np.count_nonzero(child_image != robot_image)/20000
print(f"Hamming Distance：{hamming_distance}")

letterAcc = -1*mse*weight[0] + ssim*weight[1] - hamming_distance*weight[2]
print ("Overall_Acc", letterAcc)

# f= canvas.Canvas("feedback.pdf")
# def feedback(f):
#     f.drawString(20, 300, "Your hand writing {}")

# if letterAcc>0.3:
#     feedback(f)
#     f.showPage()
#     f.save()

if letterAcc<0.4:
    print("Your hand wrting letter:",input_text, "is not well, and it looks like:",text)




