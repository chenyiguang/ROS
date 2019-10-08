#include <cstdlib>
#include "ros/ros.h"
#include "learning_communication/AddTwoInts.h"

int main(int argc, char **argv)
{
	//初始节点
	ros::init(argc,argv,"add_two_ints_client");
	//判断参数
	if (argc != 3)
	{
		ROS_INFO("usage: add_two_inits_client X Y");
		return 1;
	}
	//添加句柄
	ros::NodeHandle n;
	//向ROS Master请求服务
	ros::ServiceClient client = n.serviceClient<learning_communication::AddTwoInts>("add_two_ints");
	learning_communication::AddTwoInts srv;
	srv.request.a = atoll(argv[1]);
	srv.request.b = atoll(argv[2]);
	//发布service请求
	if (client.call(srv))
	{
		ROS_INFO("Sum: %ld",(long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	return 0;
}
