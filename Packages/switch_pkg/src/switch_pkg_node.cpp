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
#include "std_msgs/Float64.h"
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
float ve=0.08;
float re=1;
float Pointsavgt=0;
float dt = 0.1;
float comm;
float commt;
int d1=30; //delay at begin
int d2=15; //delay between turns
int d3=-1; //delay in straight 2
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
float steerl=450;
float steerr=230;
float pixdif=0;
float ypose;
float xpose;
float setpos=0;
float yaw=0;
float yaw0=0;
float rotpos=0;
float rotyaw=0;
ros::Publisher clean_pub;
ros::Publisher commt_pub;
ros::Publisher yawp_pub;
geometry_msgs::Twist cmd;
std_msgs::Int32 nyp;
std_msgs::Int32 yawcorr;

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

void ypose_callback(const std_msgs::Float64 &ypose1){
  ypose=ypose1.data;
}

void xpose_callback(const std_msgs::Float64 &xpose1){
  xpose=xpose1.data;
}

void yaw_callback(const geometry_msgs::Vector3 &yaw1){
  yaw=yaw1.z;
}

float yawset=yaw;

double _Kp = 2.1;
double _Ki = 0.0000;
double _Kd = 0.5;
double _pre_error, _integral;

double _Kpy = 0.022;
double _Kiy = 0.0000;
double _Kdy = 0.004;
double _pre_errory, _integraly;

double calculateline( double setpoint, double pv ){
    double error = setpoint - pv;
    double Pout = _Kp * error;
    _integral += error * dt;
    double Iout = _Ki * _integral;
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;
    double output = Pout + Iout + Dout;
    _pre_error = error;
  }

double calculateyaw( double setpointy, double pvy ){
      double errory = setpointy - pvy;
      double Pouty = _Kpy * errory;
      _integraly += errory * dt;
      double Iouty = _Kiy * _integraly;
      double derivativey = (errory - _pre_errory) / dt;
      double Douty = _Kdy * derivativey;
      double outputy = Pouty + Iouty + Douty;
      _pre_errory = errory;
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
  if ((comm==1) && (counter==0)){
  counter++;
  commt=1;
  yaw0=yaw;
  }

  if (((ypose<(-0.25)) || (comm==4)) && (yaw<(yaw0+5)) && (yaw>(yaw0-5))){
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
  if ((comm==4) && (counter5==0)){
  counter5++;
  commt=4;
  yaw0=yaw;
  }
  if ((ypose>(-0.20)) && (yaw<(yaw0+5)) && (yaw>(yaw0-5)) && (xpose>0)){
    counter5=0;
    counter6=0;
    counter7=0;
    counter8=0;
    commt=0;
    yaw0=0;
  }
  if (counter9>0){
    commt=5;
  }
}

void steer(){
  if (commt==0){
    rotcleanp=rotclean;
    vcleanp=vclean;
  }
  if (commt==1){
  rotcleanp=(calculateline(ypose, -0.32))+(calculateyaw(yaw0, yaw));
  rotpos=calculateline(ypose, -0.32);
  rotyaw=calculateyaw(yaw0, yaw);
  //printf("\r\nrotpos %f",rotpos);
 //printf("\r\nrotyaw %f",rotyaw);
  if (comm==6){
  vcleanp=vadapted;
}
  else {
  vcleanp=vclean+ve;
}
}
  if (commt==3){
   rotcleanp=rotclean;
   vcleanp=0;} //Slow down
  if (commt==2){
    rotcleanp=rotclean;
    vcleanp=vadapted;
  } //Adapt
  if (commt==4){
    rotcleanp=(calculateline(ypose, -0.05))+(calculateyaw(yaw0, yaw));
    rotpos=calculateline(ypose, -0.05);
    rotyaw=calculateyaw(yaw0, yaw);
    //printf("\r\nrotpos %f",rotpos);
    //printf("\r\nrotyaw %f",rotyaw);
  if (comm==6){
  vcleanp=vadapted;
}
else {
  vcleanp=vclean+ve;
}
}
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

void needypose(){
  if ((commt==1) || (commt==4) || (commt==5)){
    nyp.data=1;
  }
  else{
    nyp.data=0;
  }
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
    ros::Subscriber ypose_sub = n.subscribe("/ycoordinate", 1, ypose_callback);
    ros::Subscriber xpose_sub = n.subscribe("/xcoordinate", 1, xpose_callback);
    ros::Subscriber yaw_sub = n.subscribe("/rpy", 1, yaw_callback);
    ros::Rate loop_rate(10);
      clean_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
      commt_pub = n.advertise<std_msgs::Int32>("/NEEDYPOSE", 1);
      yawp_pub = n.advertise<std_msgs::Int32>("/yawcorrected",1);
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z = 0;
      nyp.data=0;

    while(ros::ok()){
      vcleanp=vclean;
      rotcleanp=rotclean;
      ranges();
      state3();
      state1();
      state4();
      steer();
      opslag();
      needypose();
      yawcorr.data=(yaw-yaw0);
      //printf("\r\nyaw %f",yaw);
      cmd.linear.x = vcleanp;
      cmd.angular.z = rotcleanp;
      clean_pub.publish(cmd);
      yawp_pub.publish(yawcorr);
      commt_pub.publish(nyp);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
