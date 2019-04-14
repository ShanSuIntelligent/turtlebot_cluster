/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef INPUT_PATH_H
#define INPUT_PATH_H

#include "yikun_common/mysql/db_helper.h"

namespace yikun_common {

class InputPath
{
public:
  typedef boost::shared_ptr<yikun_common::InputPath> Ptr;
  InputPath(DbHelper::Ptr& db);
  ~InputPath();

  void supervise(const Robot& robot);
private:
  DbHelper::Ptr local_db_;
};

}//namespace

#endif // INPUT_PATH_H
