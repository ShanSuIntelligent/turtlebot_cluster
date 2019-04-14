/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include <iostream>
#include <fstream>
#include "yikun_common/mysql/db_helper.h"
#include "yikun_common/service/header.h"
#include <tf/tf.h>

using namespace yikun_common;
std::string global_frame_ = "odom";
/**
 * @brief 路径解析
 */
bool getPath(const char *p,std::vector<WayPoint> &path);

int main(int argc,char* argv[])
{
//  ros::init(argc,argv,"path_analyse");
//  ros::NodeHandle nh;
  if(argc < 2) {
    std::cout<<"Please input the path file"<<std::endl;
    exit(1);
  }
  DbHelper::Ptr db = boost::make_shared<DbHelper>();
  if(!db->dbconnect()) {
    std::cout<<"Unable to connect to db"<<std::endl;
    exit(1);
  }
  std::ifstream in(argv[1]);
  std::string data;
  for(std::string s;in>>s;)
  {
    data.append(s);
  }
  std::vector<std::string> paths;
  if(data.find('-') == std::string::npos)
    return -1;
  boost::split(paths,data,boost::is_any_of("-"));
  std::cout<<paths.size()<<std::endl;
  for(std::string pathstr : paths)
  {
    if(pathstr.find(':') == std::string::npos)
      continue;
    std::vector<std::string> tmp;
    boost::split(tmp,pathstr,boost::is_any_of(":"));
    std::string table = tmp.front();
    std::vector<WayPoint> path;
    if(!getPath(tmp.back().c_str(),path)) {
      continue;
    }
    std::cout<<table<<std::endl;
    db->emptyTable(table);
    for(auto p : path) {
      db->insertPoints(table,p);
    }
  }
  return 0;
}

bool getPath(const char *p,std::vector<WayPoint> &path)
{
  path.clear();
  //
  std::string pathstr = p;
  std::vector<std::string> points;
  if(pathstr.find(';') == std::string::npos)
    return false;
  boost::split(points,pathstr,boost::is_any_of(";"));
  for(std::string point : points)
  {
    std::vector<std::string> index;
    WayPoint pose;
    if(point.find(',') == std::string::npos)
      return true;
    boost::split(index,point,boost::is_any_of(","));
    pose.pose.header.frame_id = global_frame_;
//    pose.header.stamp = ros::Time::now();
    //
    pose.pose.pose.position.x = atof(index[0].c_str())*0.01;	//x
    pose.pose.pose.position.y = atof(index[1].c_str())*0.01;	//y
    pose.parking = atoi(index[3].c_str());
    //
//    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    path.push_back(pose);
  }
  return true;
}
