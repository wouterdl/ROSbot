
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Int64MultiArray.h>


class Linedata    //Class for the data from the Hough lines Node
{

public:

  //Initializing points and callbackfunction
  cv::Point point1;
  cv::Point point2;
  cv::Point point3;
  cv::Point point4;
  void linescallback(const std_msgs::Int64MultiArray::ConstPtr& coords);
};

void Linedata::linescallback(const std_msgs::Int64MultiArray::ConstPtr& coords)
{
  //Putting message in array and converting to cv::Mat
  int array[8] = {coords->data[0], coords->data[1], coords->data[2], coords->data[3], coords->data[4], coords->data[5], coords->data[6], coords->data[7]};
  cv::Mat Points(1, 8, CV_32SC1, array);

  //creating the points of the lines
  point1 = {Points.at<int>(0,0), Points.at<int>(0,1)};
  point2 = {Points.at<int>(0,2), Points.at<int>(0,3)};
  point3 = {Points.at<int>(0,4), Points.at<int>(0,5)};
  point4 = {Points.at<int>(0,6), Points.at<int>(0,7)};
}

class Imagedata   //CLass for the data from the Image feed
{

public:

  //Initializing cv images and callback function
  cv_bridge::CvImagePtr imageCV;
  cv::Mat imageCV2;
  void imagecallback(const sensor_msgs::ImageConstPtr& image);
};

void Imagedata::imagecallback(const sensor_msgs::ImageConstPtr& image)
{
  //Converting image message to CV image
  try
    {
    imageCV = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); //convert ros image format to cv image
    }
  catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exeption %s", e.what());
    return;
    }
    imageCV2 = imageCV->image;    //extracting pure image data from imageCV

}

int main(int argc, char **argv)
{

  //Initializing the node and class objects
  ros::init(argc, argv, "draw_houghlines");
  ros::NodeHandle n;
  Linedata linedata;
  Imagedata imagedata;

  //Subscribing to image topic
  image_transport::ImageTransport it(n);
  image_transport::Subscriber imgsub;
  imgsub = it.subscribe("/camera/rgb/image_raw", 1, &Imagedata::imagecallback, &imagedata);

  //Subscrbing to Hough lines topic
  ros::Subscriber linesub = n.subscribe("Hough", 10, &Linedata::linescallback, &linedata);
  image_transport::Publisher imgpub = it.advertise("image_lines", 1);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    //Drawing the Hough lines on the image feed and converting to image message
    cv::line(imagedata.imageCV2, linedata.point1, linedata.point2, cv::Scalar(0,255,0), 3, -1);
    cv::line(imagedata.imageCV2, linedata.point3, linedata.point4, cv::Scalar(0,255,0), 3, -1);
    sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imagedata.imageCV2).toImageMsg();

    //publishing image message
    imgpub.publish(imgmsg);
    ros::spinOnce();
    loop_rate.sleep();
  }



}
