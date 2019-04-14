/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/mysql/db_helper.h"
namespace yikun_common {
DbHelper::DbHelper()
  : initialized_(false)
{
  dbaddr_ = "127.0.0.1";
  dbname_ = "yk";
  dbuser_ = "root";
  dbpasswd_ = "123";
  sql_ = mysql_init((MYSQL*)0);
}

DbHelper::DbHelper(const std::string& addr)
  : initialized_(false),dbaddr_(addr)
{
  dbname_ = "yk";
  dbuser_ = "root";
  dbpasswd_ = "123";
  sql_ = mysql_init((MYSQL*)0);
}

DbHelper::DbHelper(const std::string& addr,const std::string& dbname)
    : initialized_(false),
      dbaddr_(addr),dbname_(dbname)
{
    dbuser_ = "root";
    dbpasswd_ = "123";
    sql_ = mysql_init((MYSQL*)0);
}
DbHelper::~DbHelper()
{
  dbdisconnect();
}

bool DbHelper::dbconnect()
{
  if(1 == sql_->reconnect)
  {
    return true;
  }
  if(sql_ != NULL && mysql_real_connect(sql_,dbaddr_.data(),dbuser_.data(),dbpasswd_.data(),dbname_.data(),3306,NULL,0))
  {
    if(!mysql_select_db(sql_,dbname_.data()))
    {
      ROS_DEBUG("select successfully.");
      sql_->reconnect = 1;
      initialized_ = true;
    }
  }
  else
  {
    ROS_DEBUG("Uable to connect the databases: %s,%s",dbaddr_.data(),dbname_.data());
  }
  return initialized_;
}
bool DbHelper::dbdisconnect()
{
  if(sql_->reconnect == 0){
    initialized_ = false;
    return true;
  }
  mysql_close(sql_);
  initialized_ = false;
  return true;
}

bool DbHelper::emptyTable(const std::string &table)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"TRUNCATE TABLE `%s`;",table.c_str());
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }


}

bool DbHelper::getRobots(std::vector<Robot> &robots)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"SELECT `id`,`host_name`,`ip`,`port`,`input_path`,`active` FROM `robots`;");
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    Robot robot;
    robot.id = atoi(row[0]);
    robot.hostname = row[1];
    robot.addr = row[2];
    robot.port = row[3];
    robot.input_path = row[4];
    robot.active = atoi(row[5]);
    robots.push_back(robot);
  }
  mysql_free_result(res);
  return true;
}

bool DbHelper::getActiveRobots(std::vector<Robot> &robots)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"SELECT `id`,`host_name`,`ip`,`port`,`input_path` FROM `robots` WHERE active=1;");
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    Robot robot;
    robot.id = atoi(row[0]);
    robot.hostname = row[1];
    robot.addr = row[2];
    robot.port = row[3];
    robot.input_path = row[4];
    robots.push_back(robot);
  }
  mysql_free_result(res);
  return true;
}

bool DbHelper::updateRrt(const RobotsInfo &info)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"UPDATE `robots_info` SET `rrt`=%f "
                   "WHERE `robot_id`=%d;",
               info.rrt,info.robot_id);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

bool DbHelper::updateRobotsInfo(const RobotsInfo& info)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"UPDATE `robots_info` SET `state`=%d,`pos_x`=%f,`pos_y`=%f,"
                   "`theta`=%f,`vel_x`=%f,`vel_th`=%f,`battery`=%d,`charger`=%d,"
                   "`left_encoder`=%d,`right_encoder`=%d,`bumper`=%d,"
                   "`wheel_drop`=%d,`cliff`=%d,`left_pwm`=%d,`right_pwm`=%d,"
                   "`left_current`=%d,`right_current`=%d,`over_current`=%d,`rrt`=%f "
                   "WHERE `robot_id`=%d;",
               info.info.state,info.info.pos.x,info.info.pos.y,info.info.pos.theta,
               info.info.vel.vel_x,info.info.vel.vel_th,info.info.battery,info.info.charger,
               info.info.left_encoder,info.info.right_encoder,info.info.bumper,info.info.wheel_drop,
               info.info.cliff,info.info.left_pwm,info.info.right_pwm,info.info.left_current,info.info.right_current,
               info.info.over_current,info.rrt,info.robot_id);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

bool DbHelper::getRuntimeInfo(RuntimeInfo &info)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"SELECT `state`, `pos_x`, `pos_y`, `theta`, `battery`, `charger`, "
                   "`vel_x`, `vel_th`, `left_encoder`, `right_encoder`, `bumper`, `wheel_drop`, "
                   "`cliff`, `left_pwm`, `right_pwm`, `left_current`, `right_current`, `over_current` "
                   "FROM `runtime_info` WHERE 1");
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    info.state = atoi(row[0]);
    info.pos.x = atof(row[1]);
    info.pos.y = atof(row[2]);
    info.pos.theta = atof(row[3]);
    info.battery = atoi(row[4]);
    info.charger = atoi(row[5]);
    info.vel.vel_x = atof(row[6]);
    info.vel.vel_th = atof(row[7]);
    info.left_encoder = atoi(row[8]);
    info.right_encoder = atoi(row[9]);
    info.bumper = atoi(row[10]);
    info.wheel_drop = atoi(row[11]);
    info.cliff = atoi(row[12]);
    info.left_pwm = atoi(row[13]);
    info.right_pwm = atoi(row[14]);
    info.left_current = atoi(row[15]);
    info.right_current = atoi(row[16]);
    info.over_current = atoi(row[17]);
    mysql_free_result(res);
    return true;
  }
  mysql_free_result(res);
  return false;
}
bool DbHelper::updatePosToRuntimeInfo(const Position& pos)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"UPDATE `runtime_info` SET `pos_x`=%f,`pos_y`=%f,`theta`=%f WHERE 1;",
               pos.x,pos.y,pos.theta);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

bool DbHelper::updateRuntimeInfo(const RuntimeInfo& info)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"UPDATE `runtime_info` SET `pos_x`=%f,`pos_y`=%f,"
                   "`theta`=%f,`vel_x`=%f,`vel_th`=%f,`battery`=%d,`charger`=%d,"
                   "`left_encoder`=%d,`right_encoder`=%d,`bumper`=%d,"
                   "`wheel_drop`=%d,`cliff`=%d,`left_pwm`=%d,`right_pwm`=%d,"
                   "`left_current`=%d,`right_current`=%d,`over_current`=%d WHERE 1;",
               info.pos.x,info.pos.y,info.pos.theta, info.vel.vel_x,info.vel.vel_th,info.battery,info.charger,
               info.left_encoder,info.right_encoder,info.bumper,info.wheel_drop,
               info.cliff,info.left_pwm,info.right_pwm,info.left_current,info.right_current, info.over_current);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

bool DbHelper::getPoints(const std::string& table,std::vector<WayPoint>& poses)
{
  MYSQL_RES *res;
  MYSQL_ROW row;
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"SELECT `id`, `frame`, `x`, `y`,`parking`,`duration` FROM `%s`",table.c_str());
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  res = mysql_store_result(sql_);
  while(row = mysql_fetch_row(res)){
    WayPoint pos;
    pos.pose.header.stamp = ros::Time::now();
//    pos.id = atoi(row[0]);
    pos.pose.header.frame_id = row[1];
    pos.pose.pose.position.x = atof(row[2]);
    pos.pose.pose.position.y = atof(row[3]);
    pos.parking = atoi(row[4]);
    pos.duration = atof(row[5]);
    pos.pose.pose.orientation.x = 0;
    pos.pose.pose.orientation.y = 0;
    pos.pose.pose.orientation.z = 0;
    pos.pose.pose.orientation.w = 1;
    poses.push_back(pos);
  }
  mysql_free_result(res);
  return true;
}

bool DbHelper::insertPoints(const std::string& table,const WayPoint& point)
{
  char tmp[NORMALSIZE];
  std::sprintf(tmp,"INSERT INTO `%s`(`frame`, `x`, `y`,`parking`,`duration`) VALUES (\"%s\",%f,%f,%d,%f)",
               table.c_str(),point.pose.header.frame_id.c_str(),point.pose.pose.position.x,
               point.pose.pose.position.y,point.parking,point.duration);
  int ret = mysql_real_query(sql_,tmp,strlen(tmp));
  if(ret)
  {
    ROS_ERROR("%s: %s\n",__func__,mysql_error(sql_));
    return false;
  }
  else
  {
    return true;
  }
}

}//namespace
