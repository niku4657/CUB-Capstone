import cv2
from skimage import color, io
# from scipy.ndimage import gaussianfilter
# import matplotlib.pyplot as plt

# fig = plt.figure()
# ax1 = fig.add_subplot(221)
# ax2 = fig.add_subplot(222)
# ax3 = fig.add_subplot(223)
# ax4 = fig.add_subplot(224)
image = io.imread('simfield1.png') #original image
imageGray = color.rgb2gray(image)


# result1 = gaussianfilter(imageGray, sigma=50) #gaussian filtered image 1
# result2 = gaussianfilter(imageGray, sigma=3) #gaussian filtered image 2

# remove noise
result1 = cv2.GaussianBlur(imageGray,(0,0),50, 50)
result2 = cv2.GaussianBlur(imageGray,(0,0),3, 3)


DoG = result1 - result2 #difference of gaussians


# ax1.imshow(imageGray)
# ax2.imshow(result1)
# ax3.imshow(result2)
# ax4.imshow(DoG)
# plt.show()

#show image
cv2.imshow("Original", image)
cv2.waitKey(0)

#show image
cv2.imshow("blur 1", result1)
cv2.waitKey(0)

#show image
cv2.imshow("blur 2", result2)
cv2.waitKey(0)

#show image
cv2.imshow("Difference of Gaussians", DoG)
cv2.waitKey(0)
