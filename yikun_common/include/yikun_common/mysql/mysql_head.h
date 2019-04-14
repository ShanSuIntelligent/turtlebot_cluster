/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#ifndef MYSQL_HEAD_H
#define MYSQL_HEAD_H
#include <iostream>
#include <ros/ros.h>
#include <mysql/mysql.h>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <map>
#include <time.h>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>

#define NORMALSIZE 1024

/**
   * @brief 延时
   */
static void _delay(int usec)
{
  clock_t now = clock();
  while(clock()-now < usec);
}

//从机状态
enum SlaveState{
  UnKnown       = -1,
};
typedef struct position {
  float       x;		//x轴坐标位置
  float       y;		//y轴的坐标位置
  float       theta;		//z轴的旋转角度
}Position;

typedef struct velocity {
  float     vel_x;		//线速度
  float     vel_th;		//角速度
}Velocity;

//小车状态
typedef struct runtimeinfo{
  void setZero()
  {
    this->state = 1;
    this->pos.x = 0;
    this->pos.y = 0;
    this->pos.theta = 0;
    this->battery = 0;
    this->charger = 0;
    this->vel.vel_x = 0;
    this->vel.vel_th = 0;
    this->left_encoder = 0;
    this->right_encoder = 0;
    this->bumper = 0;
    this->wheel_drop = 0;
    this->cliff = 0;
    this->left_pwm = 0;
    this->right_pwm = 0;
    this->left_current = 0;
    this->right_current = 0;
    this->over_current = 0;
  }
  int       state;		//状态
  Position  pos;
  int       battery;	//电量
  int       charger;
  Velocity  vel;
  int       left_encoder;
  int       right_encoder;
  int       bumper;
  int       wheel_drop;
  int       cliff;
  int       left_pwm;
  int       right_pwm;
  int       left_current;
  int       right_current;
  int       over_current;
}RuntimeInfo;

//小车信息
typedef struct robot {
  int           id;					//小车id
  std::string   hostname;		//小车的主机名
  std::string   addr;				//小车地址
  std::string   port;				//小车端口号
  std::string   input_path;
//  std::string   uwb_record;
  bool          active;     //小车的往返时延
}Robot;
//小车信息
typedef struct robotsinfo {
  void setZero()
  {
    info.setZero();
  }

  int         robot_id;			//小车id
  float       rrt;		//时间戳
  RuntimeInfo info;					//小车信息
}RobotsInfo;

typedef struct waypoint{
  double  duration;
  bool    parking;
  geometry_msgs::PoseStamped pose;
}WayPoint;


static const std::string __input_path = "input_path";
static const std::string __INFO       = "INFO";
static const std::string __WARN       = "WARN";
static const std::string __ERROR      = "ERROR";
static const std::string __FATAL      = "FATAL";

#endif // MYSQL_HEAD_H
