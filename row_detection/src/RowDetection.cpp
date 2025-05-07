/****************************
 * Trimble Autonomous CS Capstone
 * Classical CV Row Detection
 * Row Detection Methods for the cv_row_detection ros node
 * High level documentation of algorithm can be found in CV_ALGO_README.md in
 * the ROS-Package dir on github
****************************/

#include "RowDetection.hpp"
#include "ImageProc.hpp"
#include "limits.h"
#include "tuple"
#include "vector"
#include <cmath>
using namespace cv;


/**************************
 * Given two points on the image, each representing the 
 * center of a particular row candidate
 * find the angle of the direction vector between centers and 
 * the vertical unit vector 
 * @param vector<int> PrevCenter - center of a row candidate for previous section
 * @param vector<int> RowCenter - center of a row candidate for current section
 * @return angle in radians 
 ************************/
double getAngle(std::vector<int> PrevCenter, std::vector<int> RowCenter)
{
    std::vector<double> DirVec;
    std::vector<double> VertVec;
    double DotProd, Det, Theta;

    // define direction vector
    DirVec.push_back((double)(PrevCenter.at(0) - RowCenter.at(0)));
    DirVec.push_back((double)(RowCenter.at(1) - PrevCenter.at(1)));

    //define vertical unit vector
    VertVec.push_back(0);
    VertVec.push_back(RowCenter.at(1));

    DotProd = (DirVec.at(0) * VertVec.at(0)) + (DirVec.at(1) * VertVec.at(1));

    Det = (DirVec.at(0) * VertVec.at(1)) - (DirVec.at(1) * VertVec.at(0));

    Theta = atan2(Det, DotProd);

    return Theta;
}

/***************************
 * Finds average row width in pixels of 
 * all three rows combined
 * @param v<v<int>> LeftRows @param v<v<int>> CenterRows
 * @param v<v<int>> RightRows
 * @return average width in pixels of rows
 ***************************/
int getAvgRowWidth(std::vector<std::vector<int>> LeftRows, std::vector<std::vector<int>> CenterRows, 
                        std::vector<std::vector<int>> RightRows)
{
    int AverageRowWidth = 0;
    int RowWidthSum = 0;
    for(int i = 0; i < LeftRows.size(); i++){
        RowWidthSum += LeftRows[i][1] - LeftRows[i][0];
    }
    for(int i = 0; i < RightRows.size(); i++){
        RowWidthSum += RightRows[i][1] - RightRows[i][0];
    }
    for(int i = 0; i < CenterRows.size(); i++){
        RowWidthSum += CenterRows[i][1] - CenterRows[i][0];
    }

    if ((RightRows.size() + LeftRows.size() + CenterRows.size()) != 0){
        AverageRowWidth = RowWidthSum / (RightRows.size() + LeftRows.size() + CenterRows.size());
    }

    return AverageRowWidth;
}

/********************
 * Turns image section into binary signal representing where green is found in the section
 * @param Mat image section @param int threshold
 * @return green signal
 ********************/
std::vector<int> getGreenSignal(Mat Section, int GreenThreshold){
    std::vector<int> GreenSignal;
    int rows = Section.rows;
    int cols = Section.cols;
    double GreenSum;
    
    for (int i = 0; i < cols; i++) {
        int GreenSum = 0;
        //sum column values for given column
        for (int j = 0; j < rows; j++) {
            Vec3b BgrValues = Section.at<Vec3b>(j, i);
            if ((int)BgrValues[1] > 0){ // extract green color value
                GreenSum += 1;
            }
        }
        
        // if GreenSum for column is above threshold, detect a row at given col
        if(GreenSum >= GreenThreshold){
            GreenSignal.push_back(1);
        }
        else{
            GreenSignal.push_back(0);
        }
    }

    return GreenSignal;
}

/************************
 * Gets list of candidate rows from green signal
 * @param vector Green Signal @param int MaxRowGap # pixels, if MaxRowGap is exceeded the current row will end
 * @return left, center, and right row dimensions
 */
std::vector<std::vector<int>> getRows(std::vector<int> GreenSignal, int MaxRowGap)
{
    std::vector<std::vector<int>> Rows;
    // min length of candidate rows
    int MinRowWidth = 20;

    int Row = -1;

    for(int i = 0; i < GreenSignal.size(); i++){
        if(GreenSignal[i] == 1){
            if(Row == -1){
                if (!Rows.empty() && i - Rows.at(Rows.size()-1).at(1) <= MaxRowGap){
                    Row = Rows.at(Rows.size()-1).at(0);
                    Rows.pop_back();
                }
                else{
                    Row = i;
                }
            }
        }
        else
        {
            if(Row != -1){
                if (i - Row > MinRowWidth)
                {
                    std::vector<int> TempRow;
                    TempRow.push_back(Row);
                    TempRow.push_back(i);
                    Rows.push_back(TempRow);
                    Row = -1;
                }
            }
        }
    }

    return Rows;
}

/******************
 * Filters candidate rows to determine left, center, and right rows for an image section
 * @param vector rows, @param int previous center row x cord
 * @return left, center, and right rows
 */
std::tuple<std::vector<std::vector<int>>, int> filterRows(std::vector<std::vector<int>> Rows, std::vector<int> PrevCenter, int SectionEnd, double MaxCenterAngle){
    int MidIndex = -1;
    int MinDist = std::numeric_limits<int>::max();
    std::vector<std::vector<int>> FilteredRows;
    for (int i = 0; i < Rows.size(); i++){ // finds row detected that is closest to PrevCenter

        int RowCenterX = (Rows.at(i).at(0) + Rows.at(i).at(1))/2;
        std::vector<int> RowCenter;
        RowCenter.push_back(RowCenterX);
        RowCenter.push_back(SectionEnd);
        double CenterAngle = getAngle(PrevCenter, RowCenter);

        int rowIZero = Rows.at(i).at(0);
        if (abs(CenterAngle) <= MaxCenterAngle){
            int rowCenter = (rowIZero + Rows.at(i).at(1)) / 2;
            if(abs(PrevCenter.at(0) - rowCenter) < MinDist){
                MinDist = abs(PrevCenter.at(0) - rowCenter);
                MidIndex = i;
            }
        }
    }

    if (MidIndex > 0 && MidIndex < Rows.size()-1){
        for(int i = MidIndex - 1; i < MidIndex+2; i++){
            FilteredRows.push_back(Rows.at(i));
        } 
    }

    int Center = PrevCenter.at(0);
    if(!FilteredRows.empty()){ // calc new Center point
        Center = (Rows.at(MidIndex).at(0) + Rows.at(MidIndex).at(1)) / 2;
    } 

    return {FilteredRows, Center};

}

/**********************
 * Main Driver function for row detection
 * @param Mat image 
 * @return Image with row boxes and track lines drawn
 */
Mat ComputeFrame(Mat Image){
    Size ImgSize = Image.size();
    Mat BlurredImage, MaskedImage;
    std::vector<int> GreenSignal;
    std::vector<std::vector<int>> LeftTrack, RightTrack;
    std::vector<std::vector<int>> LeftRows, CenterRows, RightRows;
    std::tuple<std::vector<std::vector<int>>, int> FilteredRows;
    std::string AvgRowWidthStr;
    double MaxCenterAngle = M_PI / 4; // max angle from center of previous mid row and center of next mid row
    Point TextOrigin(50, 50);
    double FontScale = 1.5;
    Scalar Blue(255, 0, 0);
    int FontThickness = 4;
    int MaxRowWidth = 100;
    int MinRows = 3; // minimum number of rows to be detected in a section to use detections
    int AvgRowWidth;
    int RowStart = 1000;
    int RowEnd = 150;
    int ColStart = 240;
    int SectionWidth = 1440;
    int SectionHeight = 50;
    int PrevCenterX = SectionWidth / 2; // assumes initial position of starting row is at center of image section
    int MaxRowGap = 50; // max distance between plants in same row
    int GreenThreshold = 10; // range should be between 0 and SectionHeight
    Scalar LowerMask(25, 90, 0); // Lower range for color extraction
    Scalar UpperMask(75, 255, 255); // Upper range for color extraction
    Size BlurKernel(7,7);
    int SigmaX = 0;
    int SigmaY = 0;

    BlurredImage = Image.clone();
    MaskedImage = Image.clone();

    BlurImage(Image, BlurredImage, BlurKernel, SigmaX, SigmaY);

    MaskedImage = maskImage(Image, LowerMask, UpperMask);

    for(int SectionEnd = RowStart; SectionEnd > RowEnd; SectionEnd -= SectionHeight){
        std::vector<std::vector<int>> Rows;

        Mat ImageSection = getImgSection(MaskedImage, ColStart, SectionEnd, SectionWidth, SectionHeight);

        GreenSignal = getGreenSignal(ImageSection, GreenThreshold);

        Rows = getRows(GreenSignal, MaxRowGap);

        // discards sections if too few candidate rows were found
        if (Rows.size() >= MinRows){ 
            std::vector<int> PrevCenter;
            PrevCenter.push_back(PrevCenterX);
            PrevCenter.push_back(SectionEnd - SectionHeight);
            FilteredRows = filterRows(Rows, PrevCenter, SectionEnd, MaxCenterAngle);
            PrevCenterX = std::get<1>(FilteredRows);
            Rows = std::get<0>(FilteredRows);
    
            int SectionCenterY = SectionEnd - SectionHeight/2; 

            std::vector<int> TempRow;
            TempRow = {Rows[0][0], Rows[0][1], SectionCenterY};
            LeftRows.push_back(TempRow);
            TempRow = {Rows[1][0], Rows[1][1], SectionCenterY};
            CenterRows.push_back(TempRow);
            TempRow = {Rows[2][0], Rows[2][1], SectionCenterY};
            RightRows.push_back(TempRow);
        }

        // draw row boxes
        for(int i = 0; i < Rows.size(); i++){
            rectangle(Image, Rect(Rows[i][0]+ColStart, SectionEnd, Rows[i][1] - Rows[i][0], SectionHeight), Blue, FontThickness);
        }

    }

    AvgRowWidth = getAvgRowWidth(LeftRows, CenterRows, RightRows);
    AvgRowWidthStr = "Average Crop Row Width: " + std::to_string(AvgRowWidth) + "px";
    //putText(Image, AvgRowWidthStr, TextOrigin, FONT_HERSHEY_SIMPLEX, FontScale,
    //                    Blue, FontThickness); 

    return Image;
}   

/******************
 * Driver for video
 * @param char* Input Video @param char* Output Video 
 * @param int MaxIters
 */
int RunCodeOnVideo(const char* VideoInName, const char* VideoOutName, int MaxIter){
    
    VideoCapture VideoIn(VideoInName);

    VideoWriter VideoOut(VideoOutName, VideoWriter::fourcc('m', 'p', '4', 'v'), 10, Size(1920, 1080));

    if(!VideoIn.isOpened()){
        std::cout << "ERROR: VIDEO IS NOT OPENED" << std::endl;
        return -1;
    }

    int IterNum = 0;

    while(IterNum < MaxIter){
        Mat Frame, test_img;
        VideoIn >> Frame;

        test_img = Frame.clone();

        if (Frame.empty() || IterNum > MaxIter){
            break;
        }

        Mat ProcessedFrame = ComputeFrame(Frame);
        VideoOut.write(ProcessedFrame);

        IterNum++;
    }

    return 0;
}