/*************************************
 * Authors: Trimble Autonomous CSCI 2021-2022 Capstone Team
 * Computes Classical CV Row Prediction
 * for usage see cv_row_detection README.md
 *************************************/

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include "RowDetection.hpp"

using std::placeholders::_1;

class CvRowDetection : public rclcpp::Node
{
public:
    CvRowDetection() : Node("cv_row_detection")
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
			"perception_image", 10, std::bind(&CvRowDetection::topic_callback, this, _1));
	}

private:
	rclcpp::Parameter OutStr, WriteVid;
	bool IsVidOut;
    std::string OutPath;	
	cv::VideoWriter VideoOut;

	cv_bridge::CvImagePtr CvPerceptImagePtr;

	void topic_callback(const sensor_msgs::msg::Image::SharedPtr RosImage)
	{
		RCLCPP_INFO(this->get_logger(), "Recieved RosImage");

		try
		{ // converts ROS Image to OpenCV image
			CvPerceptImagePtr = cv_bridge::toCvCopy(RosImage, 
									sensor_msgs::image_encodings::BGR8);
		}

		catch(cv_bridge::Exception& e)
		{
			RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
			return;
		}

		// computes row prediction for frame and saves image showing predictions
		cv::Mat PredictionImage = ComputeFrame(CvPerceptImagePtr->image);

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
	rclcpp::spin(std::make_shared<CvRowDetection>());
	rclcpp::shutdown();
	return 0;
}
