/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/cluster_manager/robot_thread.h"

namespace yikun_common {

RobotThread::RobotThread(DbHelper::Ptr& db)
  : local_db_(db)
{
  nh_ = new ros::NodeHandle("~");
}
RobotThread::~RobotThread()
{
}
void RobotThread::run(const Robot& robot)
{
  //mysql
  RobotsInfo robots_info;
  robots_info.setZero();
//  std::cout<<robot.addr.data()<<std::endl;
  //对应小车数据库连接
  RobotInfo::Ptr infoptr = boost::make_shared<RobotInfo>(robot);
//  float rrt = infoptr->checkCommunication();
//  robots_info.robot_id = robot.id;
//  robots_info.rrt = rrt;
//  if(rrt == -1)
//  {
//    local_db_->updateRobotsInfo(robots_info);
//    return;
//  }
  if(!infoptr->connect()) {
    ROS_ERROR("%s: Unable to connect to target db of %s",__func__,robot.hostname.data());
    return;
  }
  if(nh_->ok())
  {
    float rrt = infoptr->checkCommunication();
    if(rrt == -1)
    {
      ROS_ERROR("%s lost communication",robot.hostname.data());
    } else {
      robots_info.robot_id = robot.id;
    //  robots_info.timestamp = ros::Time::now().toSec();
      RuntimeInfo info;
      if(!infoptr->getRuntimeInfo(info)) {
        ROS_ERROR("Could not get information");
        return;
      }
      robots_info.info = info;
      robots_info.rrt = rrt;
      if(!local_db_->updateRobotsInfo(robots_info)) {
        ROS_ERROR("update infomation failed");
        return;
      }
      return;
    }
    return;
//    ros::Duration(1.0).sleep();//更新频率（1/T）
  }
  //end
  return;
}

}//namespace
