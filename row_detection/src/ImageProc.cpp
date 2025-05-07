/****************************
 * Trimble Autonomous CS Capstone
 * Classical CV Row Detection
 * Image Processing Method
****************************/

#include "ImageProc.hpp"

using namespace cv;

/****************
 * Applies Gaussian Blur
 * @todo define const for k_size, sigmas
 * @param Mat img source image @param Mat newImg destination image
*/
void BlurImage(cv::Mat Img, Mat NewImg, Size K_Size, double SigmaX, double SigmaY){
	GaussianBlur(Img, NewImg, K_Size, SigmaX, SigmaY);
}

/****************
 * Applies Sobel Transform
 * @todo There are inconsistent errors when using this method, need to resolve before integrating 
 * back into algo, will result in seg faults when used
 * @param Mat image source image 
 * @return sobel image
*/
Mat SobelFunction(Mat image) {
	Mat gray, grad;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    
    Sobel(gray, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    Sobel(gray, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
    
    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    cvtColor(grad, grad, COLOR_GRAY2BGR);
    
    return grad;
}

/****************
 * Extracts a range of colors from an image.
 * mask1 & mask2 define a hsv color range.
 * @param Mat img source image @param Scalar mask1 lower bound for color range
 * @param Scalar mask2 upper bound for color range
 * @return masked image
*/
Mat maskImage(Mat img, Scalar mask1, Scalar mask2){
    Mat maskedImage = Mat::zeros(img.size(), CV_8UC3);
    Mat mask = Mat::zeros(img.size(), CV_8UC3);
	cvtColor(img, mask, COLOR_BGR2HSV); //convert to hsv
    inRange(mask, mask1, mask2, mask);
    bitwise_and(img, img, maskedImage, mask);
    return maskedImage;
}


/***************
 * Extract Section From Image,
 * (x,y) represents top left corner of section
 * @param Mat source image @param int x @param int y @param int width @param int height
 * @return image section 
*/
Mat getImgSection(Mat image, int x, int y, int width, int height){
    return Mat(image, Rect(x, y, width, height));
}

/***************
 * Uses Bitwise And to combine images
*/
Mat combineSobelAndColorMask(Mat sobelImage, Mat colorImage){
	Mat result;
    bitwise_and(sobelImage, colorImage, result);
    return result;
}