import cv2
import numpy as np
from skimage import color

image = cv2.imread('simfield1.png',)

#show original
cv2.imshow("Original Image", image)
cv2.waitKey(0)


 # calculate Sobelx
# Output dtype = cv2.CV_8U
sob_8ux = cv2.Sobel(image,cv2.CV_8U,1,0,ksize=3)

#show image
cv2.imshow("Sobel x", sob_8ux)
cv2.waitKey(0)


# calculate Sobely
# Output dtype = cv2.CV_8U
sob_8uy = cv2.Sobel(image,cv2.CV_8U,0,1,ksize=3)

#show image
cv2.imshow("Sobel y", sob_8uy)
cv2.waitKey(0)


# cv2 Sobel sees black-to-white as positive slope and white-to-black as␣,→negative slope
# using high datatypes, taking the absolute, and converting back to CV_8U␣,→detects both edges
sobelx64f = cv2.Sobel(image,cv2.CV_64F,0,1,ksize=3)
abs_sobel64f = np.absolute(sobelx64f)
sobel_8ux = np.uint8(abs_sobel64f)

#show image
cv2.imshow("Sobel x2", sobel_8ux)
cv2.waitKey(0)


sobelx64f = cv2.Sobel(image,cv2.CV_64F,1,0,ksize=3)
abs_sobel64f = np.absolute(sobelx64f)
sobel_8uy = np.uint8(abs_sobel64f)

#show image
cv2.imshow("Sobel y2", sobel_8uy)
cv2.waitKey(0)

import numpy
import scipy
from scipy import ndimage

blurred = cv2.GaussianBlur(image,(3,3),0)

sobelxy = cv2.Sobel(src=blurred, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=3) # Combined X and Y Sobel Edge Detection

cv2.imshow("Sobel xy", sobelxy)
cv2.waitKey(0)


# imageGray = color.rgb2gray(image)
#
#
# #blur
# img_blur = cv2.GaussianBlur(imageGray,(25,25),0)
#
#
# # Sobel Edge Detection
# sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
# sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
# sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
#
#
# # Display Sobel Edge Detection Images
# cv2.imshow('Sobel X', sobelx)
# cv2.waitKey(0)
#
# cv2.imshow('Sobel Y', sobely)
# cv2.waitKey(0)
#
# cv2.imshow('Sobel X Y using Sobel() function', sobelxy)
# cv2.waitKey(0)
