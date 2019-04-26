#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
//#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "/home/husarion/ros_workspace/src/image_proc_template/src/include/LaneDetector.hpp"
#include "/home/husarion/ros_workspace/src/image_proc_template/src/LaneDetector.cpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>



void imagecallback(const sensor_msgs::ImageConstPtr& msg)
{

cv_bridge::CvImagePtr blabla;
try
{
	blabla = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert ros image format to cv image
}
catch (cv_bridge::Exception& e)
{
	ROS_ERROR("cv_bridge exeption %s", e.what());
	return;
}
	
LaneDetector lanedetector;  // Create the class object
cv::Mat blabla;
cv::Mat img_denoise;
cv::Mat img_edges;
cv::Mat img_mask;
cv::Mat img_lines;
std::vector<cv::Vec4i> lines;
std::vector<std::vector<cv::Vec4i> > left_right_lines;
std::vector<cv::Point> lane;


      // Denoise the image using a Gaussian filter
      img_denoise = lanedetector.deNoise(imageCV);

      // Detect edges in the image
      img_edges = lanedetector.edgeDetector(img_denoise);

      // Mask the image so that we only get the ROI
      img_mask = lanedetector.mask(img_edges);

      // Obtain Hough lines in the cropped image
      lines = lanedetector.houghLines(img_mask);

      
      // Separate lines into left and right lines
      left_right_lines = lanedetector.lineSeparation(lines, img_edges);

      // Apply regression to obtain only one line for each side of the lane
      lane = lanedetector.regression(left_right_lines, blabla);




}

int main(int argc, char **argv)
{
    	//Initializing Node
	ros::init(argc, argv, "image_proc_template");
	ros::NodeHandle n;

	//Subscriber
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1, imagecallback);

	//Publisher
	ros::Publisher Hough_pub = n.advertise<std_msgs::Float64MultiArray>("Hough", 0);
	ros::Rate loop_rate(3);
	

	int count = 0;	
	while (ros::ok())
	{
		if(lane){
		std_msgs::Float64MultiArray lanelines;
		lanelines.data = lane;
		pub.publish(lanelines);
		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	
	return 0;



}


