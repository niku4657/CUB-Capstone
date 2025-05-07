/***********************************
 * Authors: Trimble Autonomous 2021-2022 CSCI Capstone
 * This node is used for testing of cv_row_detection.cpp, dl_row_detection.cpp
 * publishes image every 1ms for testing detection nodes
 * for usage see test_publisher README.md
 ***********************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
public:
  	TestPublisher() : Node("test_publisher")
  	{
		this->declare_parameter<std::string>("in_str");
		this->declare_parameter<int>("start_frame");
		this->declare_parameter<int>("end_frame");
		this->declare_parameter<int>("frame_step");

		InStr = this->get_parameter("in_str");
		StartFrame = this->get_parameter("start_frame");
		EndFrame = this->get_parameter("end_frame");

		InputPath = InStr.as_string();
		IterStart = StartFrame.as_int();
		IterEnd = EndFrame.as_int();
		FrameStep = this->get_parameter("frame_step").as_int();

		if(IterEnd < 0) // publish whole video 
		{
			IterEnd = std::numeric_limits<int>::max();
		}

		VideoIn = cv::VideoCapture(InputPath);

		IterNum = 0;

		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("perception_image", 10);
		timer_ = this->create_wall_timer(1ms, std::bind(&TestPublisher::timer_callback, this));
  	}	

private:
	sensor_msgs::msg::Image ImageMsg;
	rclcpp::Parameter InStr, StartFrame, EndFrame;
	std::string InputPath;
	int IterStart, IterEnd, FrameStep;
	cv::VideoCapture VideoIn;
	cv::Mat Frame;
	int IterNum;
	const int ColorChannels = 3;

  	void timer_callback()
  	{
		if (!VideoIn.isOpened())
		{
			RCLCPP_ERROR(get_logger(), "COULD NOT OPEN INPUT VIDEO");
			return;
		}

		if (IterNum >= IterStart && IterNum <= IterEnd)
		{
			// step over frames
			for(int i = 0; i < FrameStep; i++){
				VideoIn >> Frame;
			}

			// next frame to be published
			VideoIn >> Frame;

			if (Frame.empty())
			{
				return;
			}
			
			// place frame in sensor msg
			sensor_msgs::fillImage(ImageMsg, 
						"bgr8",			   
						Frame.rows,
						Frame.cols,
						Frame.cols * ColorChannels,
						Frame.data);

			ImageMsg.header.frame_id = "Perception Image";
		
			RCLCPP_INFO(this->get_logger(), "Publishing Test Image!");
			publisher_->publish(ImageMsg);
		}
		else if (IterNum < IterStart) // skips to defined frame in video
		{
			VideoIn >> Frame;
		}
		else if (IterNum > IterEnd)
		{
			RCLCPP_INFO(this->get_logger(), "End Frame Reached! No Longer Publishing!");
			return;
		}

		IterNum++;
  	}
  	rclcpp::TimerBase::SharedPtr timer_;
  	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
  	rclcpp::spin(std::make_shared<TestPublisher>());
  	rclcpp::shutdown();
  	return 0;
}
