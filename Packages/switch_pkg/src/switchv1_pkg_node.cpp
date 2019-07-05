#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int32.h"
#include <iostream>

float rotclean=0;
float vclean=0;
float vcleanp=0;
float vadapted=0;
float counter=0;
float counter2=0;
float counter3=0;
float counter4=0;
float counter5=0;
float counter6=0;
float counter7=0;
float counter8=0;
float counter9=0;
float rotcleanp=0;
float vold;
float vold1;
float difft=0;
float ve=0.2;
float re=1;
float Pointsavgt=0;
float comm;
float commt;
int d1=30; //delay at begin
int d2=15; //delay between turns
int d3=-15; //delay in straight 2
int s1=0;
int s2=0;
int s3=0;
float rfla = 0;
float rfra = 0;
float rfl1 = 0;
float rfr1 = 0;
float rfl2 = 0;
float rfr2 = 0;
float rfl = 0;
float rfr = 0;
ros::Publisher clean_pub;
geometry_msgs::Twist cmd;

void distfl_callback(const sensor_msgs::Range &range){
   rfl = range.range;
}

void distfr_callback(const sensor_msgs::Range &range){
   rfr = range.range;
}

void clean_callback(const geometry_msgs::Twist &clean){
  rotclean= clean.angular.z;
  vclean= clean.linear.x;
  Pointsavgt=clean.angular.x;
}

void comm_callback(const std_msgs::Int32 &comm1){
  comm=comm1.data;
}

void adpt_callback(const geometry_msgs::Twist &adpt){
  vadapted= adpt.linear.x;
}

void steergains(){
  if (comm==6){
          s1=12;
          s2=25;
          s3=10;
  }
  else {
     s1=9;
     s2=20;
     s3=7;
  }
}

void ranges(){
  if (rfl>1){
    rfl=1;
  }
  if (rfr>1){
    rfr=1;
  }
  rfla=(rfl+rfl1+rfl2)/3;
  rfra=(rfr+rfr1+rfr2)/3;
}

void state3(){
  commt=comm;
  if (comm==4){
    counter9=0;
  }
}

void state1(){
  if (counter>0){
    commt=1;
  }
  if (comm==1){
  counter++;
  commt=1;
  }
  if (counter9>0){
    commt=5;
  }
  if ((counter4==(s1+s2+s3+d2)) || (comm==4)){
    counter=0;
    counter2=0;
    counter3=0;
    counter4=0;
    commt=5;
  }
}

void state4(){
  if (counter5>0){
    commt=4;
  }
  if (comm==4){
  counter5++;
  commt=4;
  }
  if (counter8==(s1+s2+s3+d2+d3)){
    counter5=0;
    counter6=0;
    counter7=0;
    counter8=0;
    commt=0;
  }
}

void steer(){
  if (commt==0){
    rotcleanp=rotclean;
    vcleanp=vclean;
  }
  if (commt==1){
  counter2++;
  vcleanp=vclean+ve;
  difft=Pointsavgt-330;
  if ((counter2<6) && (abs(difft)<11) || (counter2>5)){
    counter3++;

    if (counter3>0){
      counter4++;

  if ((counter4>0)&(counter4<(s1))){
    rotcleanp=re;
    vcleanp=vclean+ve;
  }
  else if ((counter4>(s1-1))&(counter4<(s1+s2))){
    rotcleanp=0;
    vcleanp=vclean+ve;
  }
  else if ((counter4>(s1+s2-1))&(counter4<(s1+s2+s3))){
    rotcleanp=-re;
    vcleanp=vclean+ve;
  }
  else if ((counter4>(s1+s2-1+s3))&(counter4<(s1+s2+s3+1))){
    rotcleanp=0;
    vcleanp=vclean+ve;
  }
  else {
    rotcleanp=rotclean;
    vcleanp=vclean+ve;
  }
  if (comm==6){
  vcleanp=vadapted;
}
else {
  vcleanp=vclean+ve;
}
}  //Loop for overtaking
}} //Overtake left
  if (commt==3){
   rotcleanp=rotclean;
   vcleanp=0;} //Slow down
  if (commt==2){
    rotcleanp=rotclean;
    vcleanp=vadapted;} //Adapt
  if (commt==4){
    counter6++;
    if (vold<3.2){
            s1=12;
            s2=25;
            s3=10;
    }
    else {
       s1=9;
       s2=20;
       s3=7;
    }

    vcleanp=vclean+ve;
    difft=Pointsavgt-330;
    if ((counter6<6) && (abs(difft)<11) || (counter6>5)){
      counter7++;

    if (counter7>0){
        counter8++;

    if ((counter8>0) &(counter8<s1)){
      rotcleanp=-re;
      vcleanp=vold;
    }
    else if ((counter8>(s1-1))&(counter8<(s1+s2+d3))){
      rotcleanp=0;
      vcleanp=vold;
    }
    else if ((counter8>(s1+d3+s2-1))&(counter8<(s1+s2+s3+d3))){
      rotcleanp=re;
      vcleanp=vold;
    }
    else if ((counter8>(-1+s1+s2+s3+d3))&(counter8<(d3+s1+s2+s3+2))){
      rotcleanp=0;
      vcleanp=vold;
    }
    else {
      rotcleanp=rotclean;
      vcleanp=vold;
    }
  }
}} //Overtake right
  if (commt==5){
    counter9++;
    rotcleanp=rotclean;
    if (comm==6){
    vcleanp=vadapted;
  }
  else {
    vcleanp=vclean+ve;
  }

  //Stay fasy}
}
}

void stop(){
  if ((rfl < 0.25) || (rfr < 0.25)){
  rotcleanp=0;
  vcleanp=0;
}
}

void opslag(){
  vold=vold1;
  vold1=vcleanp;
  rfl2=rfl1;
  rfl1=rfl;
  rfr2=rfr1;
  rfr1=rfr;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"lane_controller");
    ros::NodeHandle n("~");
    ros::Subscriber clean_sub = n.subscribe("/cl_vel", 1, clean_callback);
    ros::Subscriber adpt_sub = n.subscribe("/frontspeed_v", 1, adpt_callback);
    ros::Subscriber comm_sub = n.subscribe("/command_v", 1, comm_callback);
    ros::Subscriber distfl_sub = n.subscribe("/range/fl", 1, distfl_callback);
    ros::Subscriber distfr_sub = n.subscribe("/range/fr", 1, distfr_callback);
    ros::Rate loop_rate(10);
      clean_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z = 0;

    while(ros::ok()){
      vcleanp=vclean;
      rotcleanp=rotclean;
      steergains();
      ranges();
      state3();
      state1();
      state4();
      steer();
      opslag();
    //  stop();
    //  printf("\r\nvold %f",vold);
      cmd.linear.x = vcleanp;
      cmd.angular.z = rotcleanp;
      clean_pub.publish(cmd);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
