#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
//#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include "opencv2/opencv.hpp"
#include "/home/wouter/Documents/ros_workspace/src/image_proc_template/src/include/LaneDetector.hpp"
#include "/home/wouter/Documents/ros_workspace/src/image_proc_template/src/LaneDetector.cpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Int64MultiArray.h>


class Lanes
{

public:

	//int lane[4];
	//int imageCV22[4];
	std::vector<cv::Point> lane;
	void imagecallback(const sensor_msgs::ImageConstPtr& msg);

};

void Lanes::imagecallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr imageCV;
	try
		{
		imageCV = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert ros image format to cv image
		}
	catch (cv_bridge::Exception& e)
		{
		ROS_ERROR("cv_bridge exeption %s", e.what());
		return;
		}

	LaneDetector lanedetector;  // Create the class object

	cv::Mat imageCV2;
	cv::Mat img_denoise;
	cv::Mat img_edges;
	cv::Mat img_mask;
	cv::Mat img_lines;
	std::vector<cv::Vec4i> lines;
	std::vector<std::vector<cv::Vec4i> > left_right_lines;

	imageCV2=imageCV->image;

	//imageCV22[1]={imageCV2.at<int>(1,3)};
	//imageCV22[2]={imageCV2.at<int>(1,4)};
	//imageCV22[3]={imageCV2.at<int>(1,5)};
	//imageCV3[0] = 1;
	//imageCV3[1] = 2;
	//imageCV3[2] = 3;
	//imageCV3[3] = 4;

	      // Denoise the image using a Gaussian filter
	      img_denoise = lanedetector.deNoise(imageCV2);

	      // Detect edges in the image
	      img_edges = lanedetector.edgeDetector(img_denoise);

	      // Mask the image so that we only get the ROI
	      img_mask = lanedetector.mask(img_edges);

	      // Obtain Hough lines in the cropped image
	      lines = lanedetector.houghLines(img_mask);

	      // Separate lines into left and right lines
	      left_right_lines = lanedetector.lineSeparation(lines, img_edges);

	      // Apply regression to obtain only one line for each side of the lane
	      lane = lanedetector.regression(left_right_lines, imageCV2);

				//imageCV22[0]={img_mask.at<int>(600,700)};'

				cv::Mat lanemat(lane);

}


int main(int argc, char **argv)
{

  //Initializing Node
	ros::init(argc, argv, "image_proc_template");
	ros::NodeHandle n;
	Lanes lanes;

	//Publisher
	ros::Rate loop_rate(3);
	ros::Publisher Hough_pub = n.advertise<std_msgs::Int64MultiArray>("Hough", 0);

	//Subscriber
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub;
	sub = it.subscribe("/videofile/image_raw", 1, &Lanes::imagecallback, &lanes);

	while (ros::ok())
	{
		//Publishing the message
		int lanes2[] = {lanes.lanemat.at<int>(0,0), lanes.lanemat.at<int>(0,1), lanes.lanemat.at<int>(0,2), lanes.lanemat.at<int>(0,3)};
		//int lanes2[] = {1,2,3,4};
		//std::vector<int> lanes3 (std::begin(lanes2), std::end(lanes2));
		std_msgs::Int64MultiArray lanelines;
		lanelines.data.clear();
		lanelines.layout.dim.push_back(std_msgs::MultiArrayDimension());
		lanelines.layout.dim[0].size = 4;
		lanelines.layout.dim[0].stride = 8;
		lanelines.layout.dim[1].size = 2;
		lanelines.layout.dim[1].stride = 2;


		lanelines.data.insert(lanelines.data.begin(), lanes.lanemat.begin(), lanes.lanemat.end());

		Hough_pub.publish(lanelines);

		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;

}
