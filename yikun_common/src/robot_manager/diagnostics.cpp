/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <ros/ros.h>
#include "yikun_common/mysql/db_helper.h"
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <tf/tf.h>

//#if 0
//uint8 BUMPER_RIGHT=1
//uint8 BUMPER_CENTRE=2
//uint8 BUMPER_LEFT=4
//uint8 WHEEL_DROP_RIGHT=1
//uint8 WHEEL_DROP_LEFT=2
//uint8 CLIFF_RIGHT=1
//uint8 CLIFF_CENTRE=2
//uint8 CLIFF_LEFT=4
//uint8 BUTTON0=1
//uint8 BUTTON1=2
//uint8 BUTTON2=4
//uint8 DISCHARGING=0
//uint8 DOCKING_CHARGED=2
//uint8 DOCKING_CHARGING=6
//uint8 ADAPTER_CHARGED=18
//uint8 ADAPTER_CHARGING=22
//uint8 OVER_CURRENT_LEFT_WHEEL=1
//uint8 OVER_CURRENT_RIGHT_WHEEL=2
//uint8 OVER_CURRENT_BOTH_WHEELS=3
//uint8 DIGITAL_INPUT0=1
//uint8 DIGITAL_INPUT1=2
//uint8 DIGITAL_INPUT2=4
//uint8 DIGITAL_INPUT3=8
//uint8 DB25_TEST_BOARD_CONNECTED=64
//uint16 time_stamp
//uint8 bumper
//uint8 wheel_drop
//uint8 cliff
//uint16 left_encoder
//uint16 right_encoder
//int8 left_pwm
//int8 right_pwm
//uint8 buttons
//uint8 charger
//uint8 battery
//uint16[] bottom
//uint8[] current
//uint8 over_current
//uint16 digital_input
//uint16[] analog_input
//#endif
using namespace yikun_common;
RuntimeInfo ginfo_;
boost::mutex mutex_sensor,mutex_odom,mutex_uwb;

void sensor_callback(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_sensor);
  ginfo_.bumper = msg->bumper;
  ginfo_.wheel_drop = msg->wheel_drop;
  ginfo_.cliff = msg->cliff;
  ginfo_.left_encoder = msg->left_encoder;
  ginfo_.right_encoder = msg->right_encoder;
  ginfo_.left_pwm = msg->left_pwm;
  ginfo_.right_pwm =  msg->right_pwm;
  ginfo_.charger = msg->charger;
  ginfo_.battery = msg->battery;
  ginfo_.left_current = msg->current[0];
  ginfo_.right_current = msg->current[1];
  ginfo_.over_current = msg->over_current;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_odom);
  ginfo_.vel.vel_x = msg->twist.twist.linear.x;
  ginfo_.vel.vel_th = msg->twist.twist.angular.z;
}
void uwb_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_uwb);
  ginfo_.pos.x = msg->pose.pose.position.x;
  ginfo_.pos.y = msg->pose.pose.position.y;
  ginfo_.pos.theta = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc,char*argv[])
{
  ros::init(argc,argv,"diagnostics");
  ros::NodeHandle nh;
  ros::Subscriber sensor = nh.subscribe("/mobile_base/sensors/core",1,sensor_callback);
  ros::Subscriber odom = nh.subscribe("/odom",1,odom_callback);
  ros::Subscriber uwb = nh.subscribe("/uwb_odom",1,uwb_callback);
  DbHelper::Ptr db = boost::make_shared<DbHelper>();
  while(ros::ok() && !db->dbconnect()) {
    sleep(1);
  }
  while(ros::ok())
  {
    db->updateRuntimeInfo(ginfo_);
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  return 0;
}
