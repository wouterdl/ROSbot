#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Char.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <math.h>
#include <string>

bool front_v {false};
bool dangle_v {false};
bool static_v  {false};
bool inputcheck {false};
bool cleartogo {false};
bool returncheck {false};
bool leftlane {false};
bool speedadapt {false};
bool idcheck {false};
bool overtake {false};
int needypose {0};

float angle_v;
float angle_b;
float angle_d;

bool dangle_v1;
bool dangle_v2;
bool dangle_v3;
bool danglecheck_v;
bool front_v1;
bool front_v2;
bool front_v3;
bool frontcheck_v;

float velo_d {0};
float velo_v {0};
float velo_b {0};
float dspeed {0};

float ydist {0};
float ydist1 {0};
float ydist2 {0};
float xdist {0};

float dist_b {0};
float dist_v {0};
float dist_d {0};
float ydiff {0};


float xposition;
float yposition;
float rel_vel_d;
float rel_vel_f;
float xmin;
double x1;
float x2;
float t_overtake;

char ros_id;

int size_v;
int size_d;
int size_b;

ros::Publisher com_pub_v;
ros::Publisher frontspeed_pub_v;
ros::Publisher rosid_pub;
geometry_msgs::Twist frontspeed;
std_msgs::Int32 command_v;

ros::Publisher ycoordinate_pub;
std_msgs::Float64 ycoordinate;

ros::Publisher xcoordinate_pub;
std_msgs::Float64 xcoordinate;

geometry_msgs::PoseArray vehicles_filtered_v; //main vehicle
geometry_msgs::PoseArray vehicles_filtered_d; //in front
geometry_msgs::PoseArray vehicles_filtered_b; //left lane

float yaw;

void yaw_callback(const std_msgs::Int32 &integ){
  yaw = integ.data;
}

void vehicle_callback_v(const geometry_msgs::PoseArray &vehic){
  vehicles_filtered_v.poses = vehic.poses;
}
void vehicle_callback_d(const geometry_msgs::PoseArray &vehic){
  vehicles_filtered_d.poses = vehic.poses;
}
void vehicle_callback_b(const geometry_msgs::PoseArray &vehic){
  vehicles_filtered_b.poses = vehic.poses;
}
void speed_callback_d(const geometry_msgs::Twist &vel){
  velo_d = vel.linear.x;
}
void speed_callback_v(const geometry_msgs::Twist &vel){
  velo_v = vel.linear.x;
}
void speed_callback_b(const geometry_msgs::Twist &vel){
  velo_b = vel.linear.x;
}
void needypose_callback(const std_msgs::Int32 &integer){
  needypose = integer.data;
}


void sizes()                 //checks for how many objects are nearby
{
  size_v = vehicles_filtered_v.poses.size();
  size_d = vehicles_filtered_d.poses.size();
  size_b = vehicles_filtered_b.poses.size();
}

void FrontCheck_v()                     //checks for objects in front
{
for (unsigned int i = 0; i<size_v; ++i){
  angle_v = vehicles_filtered_v.poses[i].orientation.y;
  dist_v = vehicles_filtered_v.poses[i].orientation.x;
  if ((abs(angle_v) < 10) && (abs(dist_v)<0.9)) {

    front_v = true;

  }
}

}

void deadangle_v()  //checks for objects behind on the left lane. Not being used because the predictor is better (also uses speed)
{

  for (unsigned int i = 0; i<size_v; ++i){
    angle_v = vehicles_filtered_v.poses[i].orientation.y;
    if((-35>angle_v) && (angle_v>-155)){

      for (unsigned int j = 0; j<size_d; ++j){
        angle_d = vehicles_filtered_d.poses[j].orientation.y;

        if((165 < abs(angle_v-angle_d)) &&  (abs(angle_v-angle_d) < 195)){
          dangle_v = true;
          dspeed = velo_d;
        }
      }

      for (unsigned int k = 0; k<size_b; ++k){
        angle_b = vehicles_filtered_b.poses[k].orientation.y;

        if((165 < abs(angle_v-angle_b)) &&  (abs(angle_v-angle_b) < 195)){
          dangle_v = true;
          dspeed = velo_b;
        }
      }
    }
  }
}

void predictor()  //estimate whether there is enough time for an overtaking maneuver
{

  for (unsigned int i = 0; i<size_v; ++i){
    angle_v = vehicles_filtered_v.poses[i].orientation.y;
    xposition = vehicles_filtered_v.poses[i].position.x;
    yposition = vehicles_filtered_v.poses[i].position.y;
    if((-40>angle_v) && (angle_v>-160)){

      for (unsigned int j = 0; j<size_d; ++j){
        angle_d = vehicles_filtered_d.poses[j].orientation.y;

        if((165 < abs(angle_v-angle_d)) &&  (abs(angle_v-angle_d) < 195)){
          dspeed = velo_d;
          dangle_v = true;
          x2 = -xposition;

        }
      }

      for (unsigned int k = 0; k<size_b; ++k){
        angle_b = vehicles_filtered_b.poses[k].orientation.y;

        if((165 < abs(angle_v-angle_b)) &&  (abs(angle_v-angle_b) < 195)){
          dspeed = velo_b;
          dangle_v = true;
          x2 = -xposition;

        }
      }
    if (dangle_v == true){
      rel_vel_d = dspeed - velo_v;
      rel_vel_f = (velo_v + 0.2) - frontspeed.linear.x;
      x1 = 1.5;
      t_overtake = x1/rel_vel_f +0.3;
      xmin = t_overtake * rel_vel_d +0.3;

        if (x2 < xmin){
          danglecheck_v = true;
        }
    }
  }
}
}

void staticcheck()
{

  for (unsigned int i = 0; i<size_v; ++i){
    angle_v = vehicles_filtered_v.poses[i].orientation.y;
    dist_v = vehicles_filtered_v.poses[i].orientation.x;

      if ((abs(angle_v) < 10) && (abs(dist_v)<0.8)) {


      for (unsigned int j = 0; j<size_d; ++j){
        angle_d = vehicles_filtered_d.poses[j].orientation.y;

          if((165 < abs(angle_v-angle_d)) &&  (abs(angle_v-angle_d) < 195)){
            inputcheck = true;
            //ROS_INFO_STREAM(angle_d);
            static_v = false;
            frontspeed.linear.x = velo_d;
          }

      }


      for (unsigned int k = 0; k<size_b; ++k){
        angle_b = vehicles_filtered_b.poses[k].orientation.y;

          if((165 < abs(angle_v-angle_b)) && (abs(angle_v-angle_b) < 195)){
            inputcheck = true;
            //ROS_INFO_STREAM(inputcheck);
            static_v = false;
            frontspeed.linear.x = velo_b;
          }

      }
      }
      }
      if ((inputcheck == false)){
        static_v = true;
    }
  }

void IDloop()  //assigns an ID to the vehicle in front
{
    if (idcheck == false){
      for (unsigned int i = 0; i<size_v; ++i){
        angle_v = vehicles_filtered_v.poses[i].orientation.y;
        dist_v = vehicles_filtered_v.poses[i].orientation.x;

        if((7>angle_v) && (angle_v>-7) && abs(dist_v<0.9)){

          for (unsigned int j = 0; j<size_d; ++j){
            //dist_d = vehicles_filtered_d.poses[j].orientation.x;
            angle_d = vehicles_filtered_d.poses[j].orientation.y;
            if((165 < abs(angle_v-angle_d)) &&  (abs(angle_v-angle_d) < 195)){
              ros_id = 'D';
              idcheck = true;
              ROS_INFO_STREAM("ID = B");
            }
          }
          for (unsigned int k = 0; k<size_b; ++k){
            //dist_b = vehicles_filtered_b.poses[k].orientation.x;
            angle_b = vehicles_filtered_b.poses[k].orientation.y;
            if((165 < abs(angle_v-angle_b)) &&  (abs(angle_v-angle_b) < 195)){
              ros_id = 'D';
              idcheck = true;
              ROS_INFO_STREAM("ID = B");
            }
            }

            }
          }
        ydist = 0.001 ;
    }
}

void ourpose()           //returns the positional data of the overtaking vehicle as seen by the vehicle in front
{
    if (idcheck == true){

      for (unsigned int i = 0; i<size_v; ++i){
        dist_v = vehicles_filtered_v.poses[i].orientation.x;
        angle_v = vehicles_filtered_v.poses[i].orientation.y;

      if(ros_id == 'D'){

              for (unsigned int j = 0; j<size_d; ++j){
              dist_d = vehicles_filtered_d.poses[j].orientation.x;
              angle_d = vehicles_filtered_d.poses[j].orientation.y;

                if((158 < abs(angle_v-yaw-angle_d)) &&  (abs(angle_v-yaw-angle_d) < 202) && (abs(dist_d) > (abs(dist_v)-0.03)) &&  (abs(dist_d) < (abs(dist_v)+0.03))){

                  ydist = vehicles_filtered_d.poses[j].position.y;
                }
              }
      }

      if(ros_id == 'B'){

          for (unsigned int k = 0; k<size_b; ++k){
          dist_b = vehicles_filtered_b.poses[k].orientation.x;
          angle_b = vehicles_filtered_b.poses[k].orientation.y;

              if((158 < abs(angle_v-yaw-angle_b)) &&  (abs(angle_v-yaw-angle_b) < 202) && (abs(dist_b) > (abs(dist_v)-0.03)) &&  (abs(dist_b) < (abs(dist_v)+0.03))){

                ydist = vehicles_filtered_b.poses[k].position.y;

              }
          }
        }
        }
      }
}

void gapcheck()          //checks whether it is save to return to the right lane.
{
  for (unsigned int i = 0; i<size_v; ++i){
    angle_v = vehicles_filtered_v.poses[i].orientation.y;
    dist_v = vehicles_filtered_v.poses[i].orientation.x;
    if ((angle_v > 130) && (angle_v < 145)){
    cleartogo = true;
    }
  }
}

void savestate()    //a filter that makes sure the right observations are made.
{
  dangle_v3=dangle_v2;
  dangle_v2=dangle_v1;
  dangle_v1=dangle_v;

  front_v3 = front_v2;
  front_v2 = front_v1;
  front_v1 = front_v;

  if ((front_v == false) && (front_v1 == false) && (front_v2 == false) && (front_v3 == false))
  {
    frontcheck_v = false;
  }

  else {
    frontcheck_v = true;
  }

  if((dangle_v == false) && (dangle_v1 == false) && (dangle_v2 == false) && (dangle_v3 == false))
  {
    danglecheck_v = false;
  }
  else{
    danglecheck_v = true;
  }
}


void commander()               //makes a decision and publishes the command
{
  if ((frontcheck_v == true) && (danglecheck_v == false) && (returncheck == false) && (leftlane == false) && (static_v == true))
  {
    command_v.data = 1; //overtake
    overtake = true;
    returncheck = true;
  }

  else if ((frontcheck_v == true) && (danglecheck_v == true) && (static_v == false))
  {
    command_v.data = 2; //adapt speed
  }

  else if ((frontcheck_v == true) && (danglecheck_v == true) && (speedadapt == false) && (static_v==true))
  {
  command_v.data = 2; // usually =3 to indicate to stop the vehicle. but the static function sometimes doesn't return the right value at the moment.
  }

  else if ((cleartogo == true) && (returncheck == true))
  {
  command_v.data = 4; // return back to own lane
  returncheck = false;
  speedadapt = false;
  }

  else if ((frontcheck_v == true) && (danglecheck_v == false) && (static_v==false) && (leftlane == false))
  {
    if (frontspeed.linear.x < velo_v){
      command_v.data = 1;        //overtake
    }

    else {
      command_v.data = 0;   //continue
    }
  }
  else if ((frontcheck_v == true) && (danglecheck_v == false) && (static_v == true) && (returncheck==false) && (leftlane == true))
  {
  frontspeed.linear.x = dspeed;
  command_v.data = 1;
  speedadapt = true;
  returncheck = true;
  }
  //else if ((cleartogo == false) && (speedadapt == true))
  //{
  //command_v.data = 6; //adapt speed on left lane
  //}

  else
  {
    command_v.data = 0;  //continue
  }

com_pub_v.publish(command_v);
frontspeed_pub_v.publish(frontspeed);
xcoordinate.data = xdist;


ycoordinate.data = ydist;
//ycoordinate_abs.data = abs(ydist);
ycoordinate_pub.publish(ycoordinate);
//ycoordinate_abs_pub.publish(ycoordinate_abs);
xcoordinate_pub.publish(xcoordinate);
}

void reset()
{
  dangle_v = false;
  front_v = false;
  inputcheck = false;
  cleartogo = false;
  if(needypose == 0){
    //idcheck = false;
  }
}


int main(int argc, char **argv)
{
ros::init(argc, argv,"vehiclefilter");
ros::NodeHandle n("~");
ros::Subscriber veh_v_sub = n.subscribe("/vehicles_filtered_v", 1, vehicle_callback_v);
ros::Subscriber veh_d_sub = n.subscribe("/vehicles_filtered_d", 1, vehicle_callback_d);
ros::Subscriber veh_b_sub = n.subscribe("/vehicles_filtered_b", 1, vehicle_callback_b);
ros::Subscriber vel_d_sub = n.subscribe("/cmd_vel_D", 1, speed_callback_d);
ros::Subscriber vel_v_sub = n.subscribe("/cmd_vel", 1, speed_callback_v);
ros::Subscriber vel_b_sub = n.subscribe("/cmd_vel_B", 1, speed_callback_b);

ros::Subscriber idcheck_reset_sub = n.subscribe("NEEDYPOSE",1,needypose_callback);
ros::Subscriber yaw_sub = n.subscribe("/yawcorrected",1,yaw_callback);
//ros::Subscriber yaw_sub = n.subscribe("/rpy", 1, yaw_callback);

com_pub_v = n.advertise<std_msgs::Int32>("/command_v",1);
frontspeed_pub_v = n.advertise<geometry_msgs::Twist>("/frontspeed_v",1);
ycoordinate_pub = n.advertise<std_msgs::Float64>("/ycoordinate",1);
xcoordinate_pub = n.advertise<std_msgs::Float64>("/xcoordinate",1);

ros::Rate loop_rate(10);

while(ros::ok())
{

  sizes();
  FrontCheck_v();
  if (frontcheck_v == true){
      staticcheck();
  }
  predictor();
  //deadangle_v();
  IDloop();
  ourpose();
  savestate();
  gapcheck();
  commander();
  reset();
  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
