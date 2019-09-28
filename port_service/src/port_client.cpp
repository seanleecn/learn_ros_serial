/************************************
*************客户端节点****************
* 向/port_server服务端发送请请求内容：state
* 服务类型gb
*************************************/
#include "ros/ros.h"
#include "port_service/state.h"

#include<cstdio>
#include<iostream>
#include<string>
#include<sstream>
using namespace std;

#define rBUFFERSIZE 8；
unsigned char r_buffer[rBUFFERSIZE]

// CRC8校验，字节求异或
unsigned char CRC8(unsigned char *buffer)
{
    unsigned char ret=0,csum;
		if((buffer[0]==0xA5) && (buffer[1]==0x5A))
        {
		    csum = buffer[0]^buffer[1]^buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6];
		    //ROS_INFO("check sum:0x%02x",csum);
		    if(csum == buffer[7])
            {
			    ret = 1;
		    }
		    else 
		        ret =0;
	    }
    return ret;
}

// Hex2Dec
int main(void)
{
	string s1,s2;
	int a=30;
	stringstream ss;
	ss<<hex<<a;		 //10进制转成十六进制读入流中，，再以字符串输出
	ss>>s2;			
    return 0;
}

int main(int argc, char** argv)
{
    // ROS初始化
    ros::init(argc, argv, "port_client");
    ros::NodeHandle nh;

    // 创建client并连接到server
    ros::service::waitForService("/port_server");
    ros::ServiceClient client = nh.serviceClient<port_service::gb>("/port_server");
    
    // 创建发布anger和force的话题
    ros::Publisher angerforce = nh.advertise<port_service::data>("angerforce",1000);

    // 创建串口并测试
    serial::Serial ser
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    // 接收串口数据并进行处理
    while(ros::OK())
    {
        ros::spinoence();
        if(ser.available())
        {
            ROS_INFO("Reading from port");
            ser.read(r_buffer,rBUFFERSIZE);
            if(CRC8(r_buffer) != 0)
            {
                anger = r_buffer[2,3];
                force = r_buffer[4,5];
            }
        
            // 配置request数据
            port_service::gb srv;
            gb.request.state = 
            if (client.call(srv))
            {
                ROS_INFO("send command state:%s",gb.request.state);
            }
            else
            {
                ROS_ERROR("Failed to send command");
            }
        }
        memset(r_buffer,0,rBUFFERSIZE);
    }
}

