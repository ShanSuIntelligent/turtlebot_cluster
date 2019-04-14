/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <ros/ros.h>
#include "yikun_common/mysql/db_helper.h"
#include "yikun_common/cluster_manager/robot_thread.h"
#include "yikun_common/cluster_manager/input_path.h"
#include "yikun_cluster_msgs/Rpc.h"
#include "boost/thread.hpp"

using namespace yikun_common;

//global value
bool          isconnected_;//true: online;  false: offline
double        frequency_;
DbHelper::Ptr   db_;
InputPath::Ptr  input_;
boost::mutex    mutex_;

//global function
bool callback(yikun_cluster_msgs::RpcRequest& request, yikun_cluster_msgs::RpcResponse& response)
{
  boost::mutex::scoped_lock lock(mutex_);
  //小车队列
  std::vector<Robot> robots;
  if(!db_->getRobots(robots)) {
    ROS_ERROR("Could not get robots from db");
    response.ret = "Failed";
    return true;
  }
  std::cout<<"update path"<<std::endl;
  for(Robot robot : robots)
  {
    //小车信息更新同步
    if(robot.active)
      input_->supervise(robot);
  }
  response.ret = "Succeed";
  return true;
}

///main
int main(int argc,char *argv[])
{
  ros::init(argc,argv,"robot_info_sync");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param("frequency",frequency_,(double)10);

  isconnected_ = false;
  ros::ServiceServer srv = nh.advertiseService("update_path",callback);
  db_.reset();
  db_ = boost::make_shared<DbHelper>();
  while(!db_->dbconnect()){
    ROS_ERROR("Unable to connect to mysql");
    sleep(1);
  }
//  ros::Rate loop(frequency_);
  input_ = boost::make_shared<InputPath>(db_);
  std::cout<<"Cluster Config Ready"<<std::endl;
  ros::spin();
  return 0;
}
