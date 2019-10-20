/****************************
 * STM32-ROS通讯节点
 * 订阅cmd_ang话题获得角度
 * 订阅cmd_ang回调函数，cmd_ang得到角度后，由串口发送角度到STM32
 * 通过串口接受编码器信息
 * **************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define	sBUFFERSIZE	8//串口发送缓存长度8字节
#define	rBUFFERSIZE	8//r串口接收缓存长度8字节
unsigned char s_buffer[sBUFFERSIZE];//发送缓存区
unsigned char r_buffer[rBUFFERSIZE];//接收缓存区

// 创建serial类
serial::Serial ser;

// 数据打包，从预先定义的/cmd_ang话题获得角度
void data_pack(const geometry_msgs::Twist& cmd_ang)
{
	memset(s_buffer,0,sizeof(s_buffer));
	// 报文头
	s_buffer[0] = 0xaa;
	s_buffer[1] = 0xaa;
	// 角度(根据cmd_ang给buffer赋值)
	
	// 数据写入串口
    ser.write(s_buffer,sBUFFERSIZE);
}

// 回调函数
void callback(const geometry_msgs::Twist& cmd_ang)
{
	ROS_INFO("anger: [%f]",cmd_ang.angular.z);
	data_pack(cmd_ang);
}

int main (int argc, char** argv)
{
    // ROS初始化
    ros::init(argc, argv, "stm32ros");
    // 创建句柄
    ros::NodeHandle nh;
	//订阅/cmd_ang话题,注册回调函数
	ros::Subscriber write_sub = nh.subscribe("/cmd_ang",1000,callback);
    try
    {
		// 串口名字
        ser.setPort("/dev/pts/21");
		// 波特率
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
		// 打开串口
        ser.open();
    }
	// 获取串口信息
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
	{
        ROS_INFO_STREAM("Serial Port initialized");
    }else
	{
        return -1;
    }
	
	// 10hz频率执行程序
    ros::Rate loop_rate(10);
    while(ros::ok())
	{
        // 查看一次回调队列
		ros::spinOnce();
        if(ser.available())
		{
            ROS_INFO_STREAM("Reading from serial port");
			// 读取串口发来的数据
			ser.read(r_buffer,rBUFFERSIZE);
			// 打印数据
			for(int i=0; i<rBUFFERSIZE; i++)
            {
                // 16进制的方式打印到屏幕
                // std::cout << std::hex << (r_buffer[i] & 0xff) << " ";
                std::cout << (r_buffer[i] ) << " ";
            }
			
		}
	}
}
	

