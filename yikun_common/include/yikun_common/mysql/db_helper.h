/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef DB_HELPER_H
#define DB_HELPER_H
#include "mysql_head.h"

namespace yikun_common {

class DbHelper
{
public:
  typedef boost::shared_ptr<yikun_common::DbHelper> Ptr;
  DbHelper();
  DbHelper(const std::string& addr);
  DbHelper(const std::string& addr,const std::string& dbname);
  ~DbHelper();
  //interface
  bool dbconnect();
  bool dbdisconnect();
  bool emptyTable(const std::string& table);
  /**
   * @brief function
   * */
  //robots
  /**
   * @brief 获取小车队列
   * @param robots 小车队列
   */
  bool getRobots(std::vector<Robot> &robots);
  bool getActiveRobots(std::vector<Robot> &robots);
  //robots info
  /**
   * @brief 更新小车状态
   * @param info 小车状态 
   */
  bool updateRrt(const RobotsInfo& info);
  bool updateRobotsInfo(const RobotsInfo& info);
  //runtime info
  /**
   * @brief 获取小车状态
   * @param info 小车状态
   */
  bool getRuntimeInfo(RuntimeInfo &info);
  /**
   * @brief 更新小车位置 
   */
  bool updatePosToRuntimeInfo(const Position& pos);
  bool updateRuntimeInfo(const RuntimeInfo& info);
  //points
  bool getPoints(const std::string& table,std::vector<WayPoint>& points);
  bool insertPoints(const std::string& table,const WayPoint& point);
  //

private:
  MYSQL *sql_;
  std::string dbuser_,dbaddr_,dbpasswd_,dbname_;
  bool initialized_;
};

}//namespace

#endif // DB_HELPER_H
