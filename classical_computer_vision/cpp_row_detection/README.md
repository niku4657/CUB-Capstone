# Classical Computer Vision Algorithm

This directory contains the classical vision algorithm used in the ROS package. To use this code you will need OpenCV 4.x. The Cmake file assumes you have installed OpenCV to /usr/local/, if this is not the case you will need to update Cmakelists.txt. Below you will find a high level documentation of the algorithm, refer to the source code for more detailed comments about the functionality. You will find basic image processing methods used in the algorithm in ImageProc.xpp and the algorithm itself implemented in RowDetection.xpp.

## Overview of Algorithm
The general idea behind this algorithm is to detect rows using color extraction. We acomplish this by using a color mask, with experimentally found HSV color ranges, to extract the green out of the image. Once we have isolated the green in the image, we split the image in horizontal sections. For each section we convert it into a binary signal representing where green was found. The algorithm uses a threshold, to only detection sections of green that are "tall enough", in hope of preventing erronous detection of rows. We then turn the binary signal into a list of candidate rows, which we then filter. The filtering uses the previous center row, and so sections must be computed sequentially. We are working with the assumption that the intial positoin of the center row will be known before the algorithm is started. ComputeFrame is the main method for the algorithm and can be run directly on an image. Below is psuedocode for ComputeFrame. The performance of ComputeFrame can be tuned by modifiying the following variables:

* MaxCenterAnlge = (pi/4) - maximum angle allowed between previous and next center rows
* MinRows = 3 - will discard section if less than MinRows candidate rows are found
* RowStart = 1000 - starting row of image to use for sections
* RowEnd = 150 - last row of image to use for sections
* ColStart = 240 - starting col of image to use for sections
* SectionWidth = 1440 
* SectionHeight = 50 
* LowerMask = (25, 90, 0) - used for color detection
* UpperMask = (75, 255, 255) 
* BlurKernel = (7,7) - adjusts blur applied to image
* GreenThreshold = 10 - requires that for each column, at least this number of pixels of green must be found, for binary signal to be "on" at this column

## Algorithm Psuedocode
``` Psuedocode
ComputeFrame(Image)  
    Blur Image 
    Mask Image
    
    For Section in Image
        Get Binary Signal
        Get Candidate Rows
        Filter Rows
        Draw Row Boxes on Image
        
    Return Image
```
