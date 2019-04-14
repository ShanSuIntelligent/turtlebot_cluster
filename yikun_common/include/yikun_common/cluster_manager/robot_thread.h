/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef ROBOT_THREAD_H
#define ROBOT_THREAD_H
#include "yikun_common/mysql/db_helper.h"
#include "yikun_common/cluster_manager/robot_info.h"

namespace yikun_common {

class RobotThread
{
public:
  typedef boost::shared_ptr<yikun_common::RobotThread> Ptr;
  RobotThread(DbHelper::Ptr& db);
  ~RobotThread();

  void run(const Robot& robot);

private:
  ros::NodeHandle *nh_;
  DbHelper::Ptr local_db_;

};
}//namespace


#endif // ROBOT_THREAD_H
