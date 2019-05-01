#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Int64MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


class Line2image
{

public:

  cv::Mat imagelines;
  void imagecallback(const sensor_msgs::ImageConstPtr& image, const std_msgs::Int64MultiArray::ConstPtr& coords);

};

void Line2image::imagecallback(const sensor_msgs::ImageConstPtr& image, const std_msgs::Int64MultiArray::ConstPtr& coords)
{

  cv_bridge::CvImagePtr imageCV;



  cv::Point point1;
  cv::Point point2;
  cv::Point point3;
  cv::Point point4;

  //point1.x = coords[0];
  //point1.y = coords[4];


  try
    {
    imageCV = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); //convert ros image format to cv image
    }
  catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exeption %s", e.what());
    return;
    }

    imagelines = cv::Mat::ones(3,3, CV_8UC3);
  //imagelines = cv::line(cv::line(imageCV, point1, point2, cv::Scalar(0,255,0), 1, FILLED), point3, point4, cv::Scalar(0,255,0), 1, FILLED);


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hough_test");
  ros::NodeHandle n;
  Line2image line2image;

  //Subscribing to 2 topics and sync
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/videofile/image_raw", 1);
  message_filters::Subscriber<std_msgs::Int64MultiArray> coords_sub(n, "Hough", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, std_msgs::Int64MultiArray> sync(image_sub, coords_sub, 10);
  sync.registerCallback(boost::bind(&Line2image::imagecallback, _1, _2));

  ros::Rate loop_rate(3);

  image_transport::Imagetransport it(n);
  image_transport::Publisher pub = it.advertise("image_lanes", 1);
  sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", line2image.imagelines).toImageMsg();


  while (ros::ok())
  {
    pub.publish(imgmsg);
    ros::spinOnce();
    loop_rate.sleep();
  }


}
