#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
int main (int argc, char **argv)
{
	//ROS初始化，节点的名字不能有重复
	ros::init(argc,argv,"talker");
	//创建句柄
	ros::NodeHandle n;
	//向ROS Master注册
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);
	//设置循环的频率
	ros::Rate loop_rate(10);
	int count=0;
	while(ros::ok())
	{
		//初始化ROS的消息类型
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world" << count;
		msg.data = ss.str();
		//打印日志信息
		ROS_INFO("%s",msg.data.c_str());
		//发布消息
		chatter_pub.publish(msg);
		//处理回调函数
		ros::spinOnce();
		//休眠已匹配循环频率
		loop_rate.sleep();
		++count;
	}
	return 0;
}
