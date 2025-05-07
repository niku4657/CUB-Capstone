#ifndef ROWDETECTION_HPP
#define ROWDETECTION_HPP

#include "opencv2/opencv.hpp"

double getAngle(cv::Point PrevCenter, cv::Point RowCenter);

int getAvgRowWidth(std::vector<std::vector<int>> LeftRows, std::vector<std::vector<int>> CenterRows, 
                        std::vector<std::vector<int>> RightRows);

std::vector<int> getGreenSignal(cv::Mat Section, int Threshold);

std::vector<std::vector<int>> getRows(std::vector<int> GreenSignal, int MaxRowGap);

std::tuple<std::vector<std::vector<int>>, int> filterRows(std::vector<std::vector<int>> Rows, int PrevCenterX, int PrevCenterY, double MaxCenterAngle);

cv::Mat ComputeFrame(cv::Mat Image);

int RunCodeOnVideo(const char* VideoInName, const char* VideoOutName, int MaxIter);

#endif