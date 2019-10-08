#include "ros/ros.h"
#include "learning_communication/AddTwoInts.h"

bool add(learning_communication::AddTwoInts::Request &req,
 learning_communication::AddTwoInts::Response &res)
{
	res.sum = req.a + req.b;
	ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	ROS_INFO("sending back response: [%ld]",(long int)res.sum);
	
	return true;
}

int main(int argc, char **argv)
{
	//初始化节点
	ros::init(argc,argv,"add_two_ints_server");
	//创建节点句柄
	ros::NodeHandle n;
	//向ROS Master注册add_two_ints的服务名称和回调函数（会自动接收参数）
	ros::ServiceServer service = n.advertiseService("add_two_ints",add);
	//循环等待回调函数
	ROS_INFO("Ready to add two ints.");
	ros::spin();
}
