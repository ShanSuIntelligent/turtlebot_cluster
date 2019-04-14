/*
 *   Author: Jiayuan Sun
 *   Copyright: 2018 Shanghai Yikun Electrical Engineering Co，Ltd
 */
#include "yikun_common/service/service.h"
#include <tf/tf.h>
#include <memory>
#include <nav_msgs/Odometry.h>

namespace yikun_common {

Service::Service(ros::NodeHandle &nh)
  : nh_(nh),started_(false),global_frame_id_("map"),base_frame_id_("base_footprint"),
//    sensor_frame_id_("uwb_link"),
    vel_sub_(NULL)
{
  private_nh_ = new ros::NodeHandle("~");
  //服务地址
  private_nh_->param("addr",addr_,std::string("0.0.0.0"));
  int timesec;
  private_nh_->param("timesec",timesec,60);
  //世界坐标
  private_nh_->param("global_frame_id",global_frame_id_,std::string("map"));
  //小车坐标  
  private_nh_->param("base_frame_id",base_frame_id_,std::string("base_footprint"));
  //uwb坐标
//  private_nh_->param("sensor_frame_id",sensor_frame_id_,std::string("uwb_link"));
  //movebase action
  //路径设置请求接口定义
  //publisher
  //小车速度控制接口  
  vel_pub_ = private_nh_->advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",100);
  //UWB数据发布接口  
  //  e_stop_ = private_nh_->advertise<std_msgs::Bool>("/e_stop",1);
  plan_pub_ = private_nh_->advertise<nav_msgs::Path>("global_path",1);

  //timeout
  timeout_.tv_sec = timesec;
  timeout_.tv_usec = 0;
}

Service::~Service()
{
  //线程销毁
  vel_sub_->interrupt();
	vel_sub_->join();
  delete vel_sub_;
  vel_sub_ = NULL;
}

void Service::start()
{
  if(started_) {
    std::perror("is running");
    return;
  }
  //线程初始化
  vel_sub_ = new boost::thread(boost::bind(&Service::velocitySubcriber,this));

  started_ = true;
}

void Service::stop()
{
  if(started_) {
    return;
  }
  //线程销毁
  vel_sub_->interrupt();
  vel_sub_->join();

  delete vel_sub_;
  vel_sub_ = NULL;
}


void Service::velocitySubcriber()
{
  boost::this_thread::interruption_point();
  constexpr int listen_port = UDP_PORT;
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == -1)
  {
    std::perror("socket");
    return;
  }
  //timeout
//  if(setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
//    return;
//  if(setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout_,sizeof(timeout_)) == -1)
//    return;

  sockaddr_in client;
  socklen_t len = sizeof(client);
  sockaddr_in addr;

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = inet_addr(addr_.data());
  int ret = bind(sock, (sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
  {
    std::perror("bind");
//    goto error;
    return;
  }
  while (nh_.ok())
  {
    // get connect
//    std::cout << "Waiting udp access..." << std::endl;
//    boost::unique_lock<boost::recursive_mutex> lock(recursive_mutex_);
    int *client_sock = new int();
    _twist twist;
    std::memset(&twist,0,sizeof(_twist));
    *client_sock = recvfrom(sock, &twist, sizeof(_twist), 0,reinterpret_cast<sockaddr *>(&client), &len);
    if (*client_sock == -1 && errno == EAGAIN)
    {
      std::perror("recv");
      continue;
    }
    //response
    geometry_msgs::Twist vel;
    vel.linear.x = twist.linear_x;
    vel.angular.z = twist.angular_z;
//    std::cout<<twist.linear_x<<"\t"<<twist.angular_z<<std::endl;
    vel_pub_.publish(vel);
  }
  //end
  boost::this_thread::interruption_enabled();
  return;
//error:
//  close(sock);
//  boost::this_thread::interruption_enabled();
//  return;
}


}//namespace
