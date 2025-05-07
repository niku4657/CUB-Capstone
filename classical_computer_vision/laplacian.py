import cv2
import numpy as np

# loading image
img0 = cv2.imread('simfield1.png',)

# converting to gray scale
gray = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)

# remove noise
img = cv2.GaussianBlur(gray,(7,7),0)

# convolute with proper kernels
laplacian = cv2.Laplacian(img,cv2.CV_64F)

# plt.subplot(2,2,1),plt.imshow(img,cmap = 'gray')
# plt.title('Original'), plt.xticks([]), plt.yticks([])
# plt.subplot(2,2,2),plt.imshow(laplacian,cmap = 'gray')
# plt.title('Laplacian'), plt.xticks([]), plt.yticks([])

#show image
cv2.imshow("Original", img0)
cv2.waitKey(0)

#show image
cv2.imshow("Laplacian", laplacian)
cv2.waitKey(0)
