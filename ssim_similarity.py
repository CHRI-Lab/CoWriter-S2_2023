import pytesseract
from PIL import Image
import matplotlib.pyplot as plt
import cv2
import numpy as np
from skimage import measure
from skimage.metrics import structural_similarity
from reportlab.pdfgen import canvas

weight = [1, 1, 1, 1] # weight for OCR , MSE, SSIM, Hamming Distance

def strugg_letter(text):
    input_text = text # the word the children wanna learn
    image_pathC = '/home/shen/child.jpg' # children writing
    image_pathR = '/home/shen/robot.jpg' # robot writing
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
    a = text.strip()
    total_characters = len(a)
    # print (total_characters)
    correct_characters = sum(1 for i, j in zip(text, input_text) if i == j)
    ocr = (correct_characters / total_characters)
    # Print the result
    # print("Letter Binary:",ocr)

    # Ensure image has same shape
    if child_image.shape != robot_image.shape:
        raise ValueError("Shape of two images different")
    # MSE
    mse = cv2.mean((child_image - robot_image) ** 2)[0]/60
    print("-------------------------------------------")
    print(f"MSE:                  ||||||{mse}")
    # Structural Similarity Index，SSIM
    (score, diff) = structural_similarity(child_image, robot_image, win_size=101, full=True)
    ssim = score
    print("-------------------------------------------")
    print("SSIM:                 ||||||{}".format(score))
    # Hamming Distance
    hamming_distance = np.count_nonzero(child_image != robot_image)/20000
    print("-------------------------------------------")
    print(f"Hamming Distance：    ||||||{hamming_distance}")
    print("-------------------------------------------")
    letterAcc = ocr*weight[0]-1*mse*weight[1] + ssim*weight[2] - hamming_distance*weight[3]
    print ("Overall_Acc           ||||||", letterAcc)
    print("-------------------------------------------")

    if letterAcc<0.4:
        print("Your hand wrting letter:",input_text, "is not well, and it looks like:",text[0], "based on OCR Module")




