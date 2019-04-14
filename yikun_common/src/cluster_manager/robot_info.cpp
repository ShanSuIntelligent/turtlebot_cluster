/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/cluster_manager/robot_info.h"

namespace yikun_common {

RobotInfo::RobotInfo(const Robot robot)
  : robot_(robot)
{
  db_ = boost::make_shared<DbHelper>(robot_.addr,"yk");
}

RobotInfo::~RobotInfo()
{}

bool RobotInfo::connect()
{
  if(db_->dbconnect()) {
    return true;
  }
  return false;
}

bool RobotInfo::disconnect()
{
  if(db_->dbdisconnect()) {
    return true;
  }
  return false;
}

float RobotInfo::checkCommunication()
{
  std::string out;
  float time_delay = SystemHelper::ping(robot_.addr,out);
  return time_delay;
}
bool RobotInfo::getRuntimeInfo(RuntimeInfo &info)
{
  if(db_.use_count() == 0) {
    ROS_ERROR("db has not initialized!!!");
    return false;
  }
  if(db_->getRuntimeInfo(info)) {
      return true;
  }
  return false;
}


}//namespace
