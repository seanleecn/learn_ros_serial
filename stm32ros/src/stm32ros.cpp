/****************************
 * STM32-ROS通讯节点
 * 订阅cmd_ang话题获得角度
 * 通过串口发送角度到STM32
 * 通过串口接受编码器信息
 * **************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


#define	sBUFFERSIZE	8//send buffer size 串口发送缓存长度8字节
#define	rBUFFERSIZE	8//receive buffer size 串口接收缓存长度8字节
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

// 创建Serial类
serial::Serial ser;

// 数据打包，从cmd_ang话题获得角度
void data_pack(const geometry_msgs::Twist& cmd_vel)
{
	memset(s_buffer,0,sizeof(s_buffer));
	// 报文头
	s_buffer[0] = 0xaa;
	s_buffer[1] = 0xaa;
	// 角度
	s_buffer[2] =  ；    
    s_buffer[3] =  ；
    s_buffer[4] =  ；
    s_buffer[5] =  ；
    s_buffer[6] =  ；
	// CRC位
	s_buffer[7] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]；

    ser.write(s_buffer,sBUFFERSIZE);
}

// 接收数据分析与校验
unsigned char crc8(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	//int i;
	if((buffer[0]==0xaa) && (buffer[1]==0xaa))
    {
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6];
		if(csum == buffer[7])
        {
			ret = 1;//校验通过，数据包正确
		}
		else 
		  ret =0;//校验失败，丢弃数据包
	}
	return ret;
}

//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel){
	ROS_INFO("linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	ROS_INFO("angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
	data_pack(cmd_vel);
}

int main (int argc, char** argv)
{
    // ROS初始化
    ros::init(argc, argv, "my_serial_node");
    // 创建句柄
    ros::NodeHandle nh;

	//订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key
	ros::Subscriber write_sub = nh.subscribe("/turtle1/cmd_vel",1000,cmd_vel_callback);
	//发布里程计话题 odom
	ros::Publisher read_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
	//定义tf 对象
	static tf::TransformBroadcaster odom_broadcaster;
	//定义tf发布时需要的类型消息
	geometry_msgs::TransformStamped odom_trans;
	//定义里程计消息对象
	nav_msgs::Odometry odom;
	//定义四元数变量
	geometry_msgs::Quaternion odom_quat;
	//位置 速度 角速度
	float_union posx,posy,vx,vy,va,yaw;
	//定义时间
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    //10hz频率执行
    ros::Rate loop_rate(10);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
			ser.read(r_buffer,rBUFFERSIZE);
			/*int i;
			for(i=0;i<rBUFFERSIZE;i++)
				ROS_INFO("[0x%02x]",r_buffer[i]);
			ROS_INFO_STREAM("End reading from serial port");
			*/
			if(data_analysis(r_buffer) != 0){
				int i;
				for(i=0;i<4;i++){
					posx.cvalue[i] = r_buffer[2+i];//x 坐标
					posy.cvalue[i] = r_buffer[6+i];//y 坐标
					vx.cvalue[i] = r_buffer[10+i];// x方向速度
					vy.cvalue[i] = r_buffer[14+i];//y方向速度
					va.cvalue[i] = r_buffer[18+i];//角速度
					yaw.cvalue[i] = r_buffer[22+i];	//yaw 偏航角
				}			
				//将偏航角转换成四元数才能发布
				odom_quat = tf::createQuaternionMsgFromYaw(yaw.fvalue);
				
				//发布坐标变换父子坐标系
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_link";
				//填充获取的数据
				odom_trans.transform.translation.x = posx.fvalue;//x坐标
				odom_trans.transform.translation.y = posy.fvalue;//y坐标
				odom_trans.transform.translation.z = 0;//z坐标				
				odom_trans.transform.rotation = odom_quat;//偏航角
				//发布tf坐标变换
				odom_broadcaster.sendTransform(odom_trans);
				//获取当前时间
				current_time = ros::Time::now();
				//载入里程计时间戳
				odom.header.stamp = current_time;
				//里程计父子坐标系
				odom.header.frame_id = "odom";
				odom.child_frame_id = "base_link";
				//里程计位置数据
				odom.pose.pose.position.x = posx.fvalue;
				odom.pose.pose.position.y = posy.fvalue;
				odom.pose.pose.position.z = 0;
				odom.pose.pose.orientation = odom_quat;
				//载入线速度和角速度
				odom.twist.twist.linear.x = vx.fvalue;
				odom.twist.twist.linear.y = vy.fvalue;
				odom.twist.twist.angular.z = va.fvalue;
				//发布里程计消息
				read_pub.publish(odom);
				ROS_INFO("publish odometry");
				last_time = current_time;				
			}
			memset(r_buffer,0,rBUFFERSIZE);
        }
        loop_rate.sleep();

    }
}

