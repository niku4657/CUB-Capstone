/****************************
 * Authors: Trimble Autonomous 2021-2022 Senior Capstone
 * Computes Deep Learning Prediciton
 * for usage see dl_row_detection README.md
 ****************************/
#include <fstream>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include "ImageProc.hpp"

using std::placeholders::_1;

class DlRowDetection : public rclcpp::Node
{
public:
	DlRowDetection() : Node("dl_row_detection")
	{
		this->declare_parameter<std::string>("out_str");
		this->declare_parameter<bool>("write_vid");


		OutStr = this->get_parameter("out_str");
		WriteVid = this->get_parameter("write_vid");
		
		OutPath = OutStr.as_string();
		IsVidOut = WriteVid.as_bool();

		if (IsVidOut)
		{
			VideoOut = cv::VideoWriter(OutPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10, cv::Size(1920, 1080));
		}

		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
				"perception_image", 10, std::bind(&DlRowDetection::topic_callback, this, _1));
	}

private:
	rclcpp::Parameter OutStr, WriteVid;
	bool IsVidOut;
	std::string OutPath;
	cv_bridge::CvImagePtr CvPerceptImagePtr;
	cv::VideoWriter VideoOut;
	const int CropBottom = 1024;
	const int CropMid = 512;
	const int CropTop = 128;

	void topic_callback(const sensor_msgs::msg::Image::SharedPtr RosImage)
	{
		RCLCPP_INFO(this->get_logger(), "Recieved RosImage");

		try
		{
			CvPerceptImagePtr = cv_bridge::toCvCopy(RosImage,
					sensor_msgs::image_encodings::BGR8);
		}

		catch(cv_bridge::Exception& e)
		{
			RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
			return;
		}

		// params for defining img sections
		const int OgWidth = CvPerceptImagePtr->image.cols;
		const int OgHeight = CvPerceptImagePtr->image.rows;
		const int BottomX = int ((OgWidth / 2) - (CropBottom / 2));
		const int MidX = int ((OgWidth / 2) - (CropMid / 2));
		const int TopX = int ((OgWidth / 2) - (CropTop / 2));
		const int BottomY = 0;
		const int MidY = 0;
		const int TopY = 32;
		
		cv::imwrite("bottom_img.jpg", getImgSection(CvPerceptImagePtr->image, BottomX, BottomY, CropBottom, CropBottom));
		cv::imwrite("mid_img.jpg", getImgSection(CvPerceptImagePtr->image, MidX, MidY, CropMid, CropMid));
		cv::imwrite("top_img.jpg", getImgSection(CvPerceptImagePtr->image, TopX, TopY, CropTop, CropTop));
		
		std::string filename = "predict /home/cody/ros_ws/bottom_img.jpg /home/cody/ros_ws/top_img.jpg /home/cody/ros_ws/mid_img.jpg -f /src/row_detection/src/KeyPointDetection/predict/bot_model/model";
		std::string command = "PYTHONPATH=src/row_detection/src/KeyPointDetection python3 -m ";
		command += filename;
		system(command.c_str());
		// The current way we read predictions from the model is by reading text files
		// not great and need to find more elegant solution

		std::ifstream InFile1("bottom_img.jpg.output.txt");
		std::string X1, X2, Y1, Y2;
		getline(InFile1, X1);
		getline(InFile1, Y1);
		getline(InFile1, X2);
		getline(InFile1, Y2);
		const int bx1 = std::stoi (X1) + (OgWidth - CropBottom) / 2;
		const int bx2 = std::stoi (X2) + (OgWidth - CropBottom) / 2;
		const int by1 = std::stoi (Y1) + BottomY;
		const int by2 = std::stoi (Y2) + BottomY;

		std::ifstream InFile2("mid_img.jpg.output.txt");
		getline(InFile2, X1);
                getline(InFile2, Y1);
                getline(InFile2, X2);
                getline(InFile2, Y2);
		const int mx1 = std::stoi (X1) + (OgWidth - CropMid) / 2;
		const int mx2 = std::stoi (X2) + (OgWidth - CropMid) / 2;
		const int my1 = std::stoi (Y1) + MidY;
		const int my2 = std::stoi (Y2) + MidY;

		std::ifstream InFile3("top_img.jpg.output.txt");
		getline(InFile3, X1);
		getline(InFile3, Y1);
		getline(InFile3, X2);
		getline(InFile3, Y2);
		const int tx1 = std::stoi (X1) + (OgWidth - CropTop) / 2;
		const int tx2 = std::stoi (X2) + (OgWidth - CropTop) / 2;
		const int ty1 = std::stoi (Y1) + TopY;
		const int ty2 = std::stoi (Y2) + TopY;

		cv::Mat PredictionImage = CvPerceptImagePtr->image.clone();

		// draw predicted point on img and draw lines connecting them
		cv::line(PredictionImage, cv::Point(bx1, by1), cv::Point(mx1, my1), cv::Scalar(0, 0, 255), 10);
		cv::line(PredictionImage, cv::Point(mx1, my1), cv::Point(tx1, ty1), cv::Scalar(0, 0, 255), 10);
		cv::line(PredictionImage, cv::Point(bx2, by2), cv::Point(mx2, my2), cv::Scalar(0, 0, 255), 10);
		cv::line(PredictionImage, cv::Point(mx2, my2), cv::Point(tx2, ty2), cv::Scalar(0, 0, 255), 10);
		
		cv::circle(PredictionImage, cv::Point(bx1, by1), 0, cv::Scalar(255, 255, 255), 15);
        cv::circle(PredictionImage, cv::Point(bx2, by2), 0, cv::Scalar(255, 255, 255), 15);
		cv::circle(PredictionImage, cv::Point(mx1, my1), 0, cv::Scalar(255, 255, 255), 15);
        cv::circle(PredictionImage, cv::Point(mx2, my2), 0, cv::Scalar(255, 255, 255), 15);
		cv::circle(PredictionImage, cv::Point(tx1, ty1), 0, cv::Scalar(255, 255, 255), 15);
        cv::circle(PredictionImage, cv::Point(tx2, ty2), 0, cv::Scalar(255, 255, 255), 15);	

		if (IsVidOut)
		{
			VideoOut.write(PredictionImage);
		}
		else
		{
			cv::imwrite(OutPath, PredictionImage);
		}
		
	}
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DlRowDetection>());
	rclcpp::shutdown();
	return 0;
}
