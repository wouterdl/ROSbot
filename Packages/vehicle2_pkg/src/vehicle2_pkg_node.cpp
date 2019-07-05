#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
#include <math.h>
#define PI 3.14159265

ros::Publisher veh_pub;
float vehiclecoordx;
float AbsDistance;
float vehiclecoordy;
float angle;
geometry_msgs::PoseArray unfilteredPoses;


void vehicle_callback(const geometry_msgs::PoseArray &vehic){
  unfilteredPoses.poses = vehic.poses;
  int size = unfilteredPoses.poses.size();
  unsigned int j=0;

  geometry_msgs::PoseArray filteredPoses;

    for (unsigned int i = 0; i<size; ++i){
      vehiclecoordx = -vehic.poses[i].position.x;
      vehiclecoordy = vehic.poses[i].position.y;
      AbsDistance = pow((pow(vehiclecoordx,2)+pow(vehiclecoordy,2)),0.5);
      angle = atan2(vehiclecoordy,vehiclecoordx)*180/PI;

        if ((abs(vehiclecoordx) < 1.4) && (abs(vehiclecoordy) < 0.4)) {
          filteredPoses.poses.push_back(vehic.poses[i]);
          filteredPoses.poses[j].orientation.x = AbsDistance;
          filteredPoses.poses[j].orientation.y = angle;
          filteredPoses.poses[j].position.x = -filteredPoses.poses[j].position.x;
          ++j;
        }

  }
  veh_pub.publish(filteredPoses);
}

int main(int argc, char **argv)
{
ros::init(argc, argv,"vehiclefilter");
ros::NodeHandle n("~");
ros::Subscriber veh_sub = n.subscribe("/vehicles", 1, vehicle_callback);
veh_pub = n.advertise<geometry_msgs::PoseArray>("/vehicles_filtered",1);

ros::spin();


return 0;
}
