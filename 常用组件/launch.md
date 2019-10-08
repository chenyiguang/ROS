### launch

#### 作用

配置启动ROS Master节点、一次性启动多个节点、同时对节点进行配置。

#### 基本格式

```xml
<!-- launch为根元素，所有内容都必须包含在其中 -->
<launch> 
    <!-- 启动结点需要三个基本属性pkg,type,name -->
   	<!-- output="screen"将结点的标准输出打印到终端屏幕，默认输出为日志文件 -->
    <!-- respawn="true"复位属性，结点停止时会自动重启，默认为false -->
    <!-- ns="namespace"命名空间，为结点内相对名称添加命名空间前缀 -->
    <!-- args="arguments"结点需要输出的参数 -->
    <node pkg="package-name" type="executable-name" name="node-name" output="screen" respawn="true" required="true" ns="namespace" args="arguments"/>
    <!-- arg 
		定义launch文件内部使用的局部变量
		调用：<node name="node" pkg="package" type="type" args="$(arg arg-name)" />
	-->
    <arg name="arg-name" default="arg-value"/>
    <!-- param 
		设置共享变量parameter，当launch运行后，parameter加载到参数服务器中；
		所有的活跃的结点都可以通过ros::param::get()接口来获取parameter的值，
		用户也可以使用终端的rosparam命令获得parameter的值 
	-->
    <param name="variable-name" value="variable-value" type="type"/>
    <!-- rosparam
		设置共享变量通过配置文件方式加载到参数服务器中(command必须为load)；
	-->
    <rosparam file="filename" command="load" ns="local_costmap" />
    <!-- remap 
		取别名，同一功能及其接口不同的人有不同的命名方法，因此为了保证代码的可复用性。可以使用remap将自己		   的命名方式与功能的发布者所定义的命名进行协调。
		如：有一个结点的速度控制指令话题为/turtlebot/cmd_vel，但是我们自己的机器人订阅的速度控制话题			为/cmd_vel。将/turtlebot/cmd_vel重映射成/cmd_vel，我们的机器人就可以接收到速度控自指令
	-->
    <remap from="turtlebot/cmd_vel" to="/cmd_vel"/>
    <!-- 注意区别node中的ns参数与remap的区别 -->
    <!-- include 
		包含其它所需launch文件
	-->
    <include file="$(dirname)/other.launch" />
</launch>
```

#### 例子

通过launch同时启动“计算求和”服务，客户端使用参数调用，server向终端输出计算结果；向ROS parameter注册一个变量供全局调用，在终端上使用`rosparam`查看该变量

```xml
<launch>
	<node pkg="turtlesim" name="sim1" type="turtlesim_node" />
    <node pkg="turtlesim" name="sim2" type="turtlesim_node" />
</launch>
```



