#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <iterator>
#include "opencv2/opencv.hpp"
#include "/home/husarion/ros_workspace/src/image_proc_lane/include/LaneDetector.hpp"
#include "/home/husarion/ros_workspace/src/image_proc_lane/src/LaneDetector.cpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Int64MultiArray.h>
#include <opencv2/core/hal/interface.h>


class Lanes
{

public:
	std::vector<cv::Vec4i> lines;
	cv::Mat imageCV2;
	cv::Mat img_denoise;
	cv::Mat img_edges;
	cv::Mat img_mask;
	cv::Mat img_lines;
	cv::Mat testimg;
	std::vector<int> lanevector;
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





	std::vector<std::vector<cv::Vec4i> > left_right_lines;

	imageCV2=imageCV->image;	//taking just the image data from imageCV


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

				// Converting the lane point vector to a cv Mat matrix
				cv::Mat lanemat = cv::Mat(lane);


				// Converting cv Mat matrix to standard vector
				if (lanemat.isContinuous()) {
  			lanevector.assign((int*)lanemat.datastart, (int*)lanemat.dataend);
				} 	else {
  				for (int i = 0; i < lanemat.rows; ++i) {
    			lanevector.insert(lanevector.end(), lanemat.ptr<int>(i), lanemat.ptr<int>(i)+lanemat.cols);
  				}
				}


}


int main(int argc, char **argv)
{

  //Initializing Node
	ros::init(argc, argv, "image_proc_lane");
	ros::NodeHandle n;
	Lanes lanes;


	//Publisher
	ros::Rate loop_rate(10);
	ros::Publisher Hough_pub = n.advertise<std_msgs::Int64MultiArray>("Hough", 0);


	//Subscriber
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub;
	sub = it.subscribe("/camera/rgb/image_raw", 1, &Lanes::imagecallback, &lanes);

	image_transport::Publisher imgpub = it.advertise("edge_img", 1);

	while (ros::ok())
	{
		//Drawing the hough lines on img_mask (for testing)
		cv::Mat color_dst;
		cv::cvtColor(lanes.img_mask, color_dst, CV_GRAY2BGR);
		for( size_t i = 0; i < lanes.lines.size(); i++ )
 		{
		 cv::line( color_dst, cv::Point(lanes.lines[i][0], lanes.lines[i][1]),
				 cv::Point(lanes.lines[i][2], lanes.lines[i][3]), cv::Scalar(0,0,255), 2, 8 );
 		 	}

		//Setup multiarray message
		std_msgs::Int64MultiArray lanelines;
		lanelines.data.clear();
		lanelines.layout.dim.push_back(std_msgs::MultiArrayDimension());
		lanelines.layout.dim[0].size = 8;
		lanelines.layout.dim[0].stride = 8;

		//Inserting data of the two lines in the message and sending it
		lanelines.data.insert(lanelines.data.begin(), lanes.lanevector.begin(), lanes.lanevector.end());
		Hough_pub.publish(lanelines);

		//Publishing the image message (for testing)
		sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_dst).toImageMsg();
		imgpub.publish(imgmsg);

		ros::spinOnce();
		loop_rate.sleep();

	}

	

}
