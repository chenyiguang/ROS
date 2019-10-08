#include "ros/ros.h"
#include "std_msgs/String.h"
//将接收到的订阅信息打印出来，接收到订阅消息后会进入下面回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc,char **argv)
{
	//初始化ROS节点
	ros::init(argc,argv,"listener");
	//创建节点句柄
	ros::NodeHandle n;
	//在ROS Master中注册，订阅chatter的话题，注册回调函数
	ros::Subscriber sub = n.subscribe("chatter",1000,chatterCallback);
	//循环等待回调函数
	ros::spin();
	return 0;
}
