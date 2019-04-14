/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/cluster_manager/input_path.h"

namespace yikun_common {
InputPath::InputPath(DbHelper::Ptr& db)
  : local_db_(db)
{}

InputPath::~InputPath()
{}

void InputPath::supervise(const Robot &robot)
{
  DbHelper::Ptr db = boost::make_shared<DbHelper>(robot.addr);
  if(!db->dbconnect()) {
    return;
  }
  std::vector<WayPoint> path;
  local_db_->getPoints(robot.input_path,path);

  db->emptyTable(__input_path);
  for(auto p : path)
  {
    db->insertPoints(__input_path,p);
  }
}

}//namespace
