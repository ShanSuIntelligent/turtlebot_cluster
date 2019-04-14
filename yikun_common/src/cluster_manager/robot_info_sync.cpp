/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <ros/ros.h>
#include "yikun_common/mysql/db_helper.h"
#include "yikun_common/cluster_manager/robot_thread.h"
#include "yikun_common/cluster_manager/input_path.h"

using namespace yikun_common;

//global value
bool          isconnected_;//true: online;  false: offline
double        frequency_;
//std::vector<RobotThread::Ptr> threadptr_;//小车线程队列

//global function
void sendtoMaster();

///main
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"robot_info_sync");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param("frequency",frequency_,(double)10);

  isconnected_ = false;
  DbHelper::Ptr db;
  db.reset();
  db = boost::make_shared<DbHelper>();
  while(!db->dbconnect()){
    ROS_ERROR("Unable to connect to mysql");
    sleep(1);
  }
//  ros::Rate loop(frequency_);
  RobotThread::Ptr thread = boost::make_shared<RobotThread>(db);
  std::cout<<"Cluster Config Ready"<<std::endl;
  while(nh.ok())
  {
    //小车队列
    std::vector<Robot> robots;
    if(!db->getRobots(robots)) {
      ROS_ERROR("Could not get robots from db");
      continue;
    }
    for(Robot robot : robots)
    {
      //小车信息更新同步
      std::string out;
//      std::cout<<robot.addr<<std::endl;
      float rrt = SystemHelper::ping(robot.addr,out);
      RobotsInfo info;
      info.robot_id = robot.id;
      info.rrt = rrt;
      if(rrt == -1)
      {
        db->updateRrt(info);
      } else {
        thread->run(robot);
      }
    }
    ros::Duration(0.2).sleep();
  }
  ros::spin();
  return 0;
}
