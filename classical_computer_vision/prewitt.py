import cv2
import numpy as np

img = cv2.imread('simfield1.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_gaussian = cv2.GaussianBlur(gray,(7,7),0)


#prewitt
kernelx = np.array([[1,1,1],[0,0,0],[-1,-1,-1]])
kernely = np.array([[-1,0,1],[-1,0,1],[-1,0,1]])
img_prewittx = cv2.filter2D(img_gaussian, -1, kernelx)
img_prewitty = cv2.filter2D(img_gaussian, -1, kernely)


cv2.imshow("Original Image", img)
cv2.waitKey(0)
cv2.imshow("Prewitt X", img_prewittx)
cv2.waitKey(0)
cv2.imshow("Prewitt Y", img_prewitty)
cv2.waitKey(0)
cv2.imshow("Prewitt", img_prewittx + img_prewitty)
cv2.waitKey(0)
