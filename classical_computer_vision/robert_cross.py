import cv2
import numpy as np
from scipy import ndimage
from skimage import io

roberts_cross_v = np.array( [[ 0, 0, 0 ],
[ 0, 1, 0 ],
[ 0, 0,-1 ]] )
roberts_cross_h = np.array( [[ 0, 0, 0 ],
[ 0, 0, 1 ],
[ 0,-1, 0 ]] )


#Read and convert to grayscale
img = io.imread('./simfield1.png')
img0 = cv2.imread('simfield1.png',)
img = img.astype('float64')
gray = np.dot(img[...,:3], [0.2989, 0.5870, 0.1140])
gray /= 255


# plt.imshow(gray)
# plt.show()


#get vertical and horizontal edged image
vertical = ndimage.convolve( gray, roberts_cross_v )
horizontal = ndimage.convolve( gray, roberts_cross_h )


#implement the equation
edged_img = np.sqrt( np.square(horizontal) + np.square(vertical))
#see the image
# plt.imshow(edged_img, cmap=plt.get_cmap('gray'))
# plt.show()


#show image
cv2.imshow("Original", img0)
cv2.waitKey(0)

#show image
cv2.imshow("Robert Cross", edged_img)
cv2.waitKey(0)
