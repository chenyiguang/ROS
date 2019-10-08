### 常用组件——tf

#### TF概述

实际意义是：有些时候，我们需要根据每个对象来建立坐标系，为此，当有n个对象就可能建立n个坐标系。那么，在需要两个对象进行通信（位置关系）时，两个对象如何衡量彼此的位置呢？这时，我们需要一个可供两个对象同时参考的坐标系（全局坐标系），标记两个对象各自与全局坐标系的转化关系。这时，两者可以互相辨识对方的位置。

#### 乌龟例程中的TF例程

```shell
# 切换到catkin_ws工作目录下
$ pwd
/home/chenyiguang/catkin_ws
# 通过launch文件启动这个服务例程
$ roslaunch turtle_tf turtle_tf_demo.launch
# 启动小乌龟键盘控制客户端
$ rosrun turtlesim turtle_teleop_key
```

该服务例程简要介绍：打开画布（服务端）（默认一只乌龟），创建另一只乌龟（客户端：位置信息由tf变换得到）；下图为三个坐标系之间的关系。

![turtle](img\\frames.jpg)

#### 创建tf

创建功能包

```shell
$ pwd
/home/chenyiguang/catkin_ws/src
# 创建功能包
$ catkin_create_pkg learning_tf std_msgs rospy roscpp
```

创建源码代码

```shell
$ pwd
/home/chenyiguang/catkin_ws/src/learning_tf/src
# 创建tf广播
$ touch turtle_tf_broadcaster.cpp
$ cat turtle_tf_broadcaster.cpp 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
	static tf::TransformBroadcaster br;
	
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));
	tf::Quaternion q;
	q.setRPY(0,0,msg->theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv){
	//初始化结点
	ros::init(argc, argv, "my_tf_broadcaster");
	if(argc!=2)
	{
		ROS_ERROR("need turtle name as argument");
		return -1;
	}
	turtle_name = argv[1];
	ros::NodeHandle node;
	//订阅海龟pos信息 队伍长度为10
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose",10,&poseCallback);
	ros::spin();
	return 0;
}
# 创建tf监听器
$ touch turtle_tf_listener.cpp
$ cat turtle_tf_listener.cpp
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_tf_listener");
	ros::NodeHandle node;;
	ros::service::waitForService("spawn");
	ros::ServiceClient add_turtle = 
	node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn srv;
	add_turtle.call(srv);
	
	ros::Publisher turtle_vel = 
	node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",10);
	tf::TransformListener listener;
	ros::Rate rate(10.0);
	while(node.ok()){
		tf::StampedTransform transform;
		try
		{
			listener.waitForTransform("/turtle2","/turtle1",ros::Time(0),ros::Duration(3.0));
			listener.lookupTransform("turtle2","/turtle1",ros::Time(0),transform);	
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),transform.getOrigin().x());
		//沿着两点间前进
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(),2)+pow(transform.getOrigin().y(),2));
		turtle_vel.publish(vel_msg);
		rate.sleep();
	}
	return 0;
}
```

修改CMakeLists.txt

```shell
$ pwd
/home/chenyiguang/catkin_ws/src/learning_tf
$ vim CMakeLists.txt
...
# 添加tf和turlesim功能包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  turtlesim
)
...
###########
## Build ##
###########
...
add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)

target_link_libraries(turtle_tf_broadcaster
  ${catkin_LIBRARIES}
)

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)

target_link_libraries(turtle_tf_listener
  ${catkin_LIBRARIES}
)
...
```

修改package.xml，添加下面内容

```shell
  <build_depend>tf</build_depend>
  <build_depend>turtlesim</build_depend>
  <exec_depend>tf</exec_depend>
  <exec_depend>turtlesim</exec_depend>
```

编译运行：

```shell
$ pwd
/home/chenyiguang/catkin_ws
$ catkin_make
```

编写launch文件

```shell
$ pwd
/home/chenyiguang/catkin_ws/src/learning_tf
$ mkdir launch
$ cd launch
$ vim learning_tf.launch
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="sim"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen" />
	<node pkg="learning_tf" type="turtle_tf_broadcaster" args="/turtle1" name="turtle1_tf_broadcaster" />
	<node pkg="learning_tf" type="turtle_tf_broadcaster" args="/turtle2" name="turtle2_tf_broadcaster" />
	<node pkg="learning_tf" type="turtle_tf_listener" name="listener" />
</launch>
```

