#ifndef IMAGEPROC_HPP
#define IMAGEPROC_HPP

#include "opencv2/opencv.hpp"

void BlurImage(cv::Mat Img, cv::Mat NewImg, cv::Size K_Size, double SigmaX, double SigmaY);

cv::Mat SobelFunction(cv::Mat image);

cv::Mat maskImage(cv::Mat img, cv::Scalar mask1, cv::Scalar mask2);

cv::Mat getImgSection(cv::Mat image, int x, int y, int width, int height);

cv::Mat combineSobelAndColorMask(cv::Mat sobelImage, cv::Mat colorImage);

#endif