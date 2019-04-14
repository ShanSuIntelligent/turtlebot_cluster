/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_navigation/local_panner/path_follower.h"
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include "yikun_cluster_msgs/Rpc.h"
#include "yikun_common/mysql/db_helper.h"

using namespace yikun_navigation;

typedef struct sigpath{
  double duration;
  std::vector<geometry_msgs::PoseStamped> path;
}SigPath;

bool start_ = false;
bool done_ = false;

boost::mutex lock_;
PathFollower::Ptr path_follower_;
std::vector<SigPath> pathV_;
/**
 * @brief 路径监听
 */
bool followPathCallback(yikun_cluster_msgs::Rpc::Request& request,
                  yikun_cluster_msgs::Rpc::Response& response)
{
  boost::mutex::scoped_lock lock(lock_);
  if(start_)
  {
    response.ret = "IsRunning";
    return true;
  }
  yikun_common::DbHelper::Ptr db = boost::make_shared<yikun_common::DbHelper>();
  if(!db->dbconnect()){
    response.ret = "Failed";
    return true;
  }
  std::string table = "input_path";
  std::vector<WayPoint> path;
  if(db->getPoints(table,path))
  {
    std::vector<geometry_msgs::PoseStamped> points;
    SigPath goal;
    goal.duration = 10;
    goal.path.push_back(path.front().pose);
    points.clear();
    pathV_.clear();
    for(WayPoint p : path)
    {
      points.push_back(p.pose);
      if(p.parking){
        SigPath sp;
        sp.path = points;
        sp.duration = p.duration;
        pathV_.push_back(sp);
        points.clear();
      }
    }

    pathV_.push_back(goal);
    response.ret = "Succeed";
    start_ = true;
    done_ = false;
    return true;
  }
  return true;
}

void publishZeroVelocity(ros::Publisher& pub)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    pub.publish(cmd_vel);
}


int main(int argc,char* argv[])
{
  ros::init(argc,argv,"path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double duration;
  private_nh.param<double>("duration",duration,10);
  yikun_common::DbHelper::Ptr db = boost::make_shared<yikun_common::DbHelper>();
  if(!db->dbconnect()){
    exit(1);
  }
  ros::ServiceServer srv = nh.advertiseService("/set_path",followPathCallback);
  //速度发布接口定义
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  tf::TransformListener tf(ros::Duration(10));
  path_follower_ = boost::make_shared<PathFollower>();
  path_follower_->initialize(&tf);
  ros::Rate loop(20);
  while(nh.ok())
  {
    if(start_ && !done_)
    {
      ROS_INFO("%d",pathV_.size());
      for(SigPath path : pathV_)
      {
        path_follower_->setPlan(path.path);
        while(nh.ok())
        {
          if(path_follower_->reached || path_follower_->isGoalReached()){
            ROS_INFO_STREAM("done");
            break;
          }
          //速度计算
          geometry_msgs::Twist twist;
          if(path_follower_->computeVelocityCommands(twist)) {
            vel_pub.publish(twist);
          } else {
            publishZeroVelocity(vel_pub);
            ROS_WARN_STREAM("fail to get velocity");
          }
          ros::spinOnce();
          loop.sleep();
        }
        ros::spinOnce();
        ROS_INFO("Pause %.4f sec",path.duration);
        ros::Duration(path.duration).sleep();
      }
      start_ = false;
      done_ = true;
    }
    ros::spinOnce();
    loop.sleep();
  }
  ros::spin();
  return 0;
}
