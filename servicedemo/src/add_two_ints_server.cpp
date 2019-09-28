#include "ros/ros.h"
#include "servicedemo/AddTwoInts.h"
//boo型函数提供两个int值求和的服务
//int值从request里面获取，而返回数据装入response内，这些数据类型都定义在srv文件内部
bool add(servicedemo::AddTwoInts::Request  &req,
         servicedemo::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv){
  //ROS节点初始化  
  ros::init(argc, argv, "add_two_ints_server");
  //创建句柄
  ros::NodeHandle n;

//发布服务，使用add函数处理
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}