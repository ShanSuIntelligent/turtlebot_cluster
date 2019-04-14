/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef SERVICE_H
#define SERVICE_H

#include "yikun_common/service/header.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "yikun_common/mysql/db_helper.h"

namespace yikun_common {
class Service
{
public:
  Service(ros::NodeHandle &nh);
  ~Service();
  /**
   * @brief 开始线程 
   */
  void start();
  /**
   * @brief 结束线程 
   */
  void stop();

  /**
   * @brief 设置路径
   * @param p 路径
   */
  bool setPath(const char *p);
private:
  //communication interface
  /**
   * @brief 速度监听线程
   */
  void velocitySubcriber();//wait velocity(UDP)

  ros::NodeHandle nh_;
  ros::NodeHandle *private_nh_;
  //线程
  boost::thread *vel_sub_;
  
  ros::Publisher vel_pub_;
  ros::Publisher plan_pub_;
  bool is_navigation_,started_;
  bool interrupt_;//中断
  std::string addr_;
  boost::recursive_mutex recursive_mutex_;
  boost::mutex mutex_;
  struct timeval timeout_;
  std::string global_frame_id_,base_frame_id_,sensor_frame_id_;
};

}//namespace

#endif // SERVICE_H
