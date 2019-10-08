/************************************
*************客户端节点****************
* 向/port_server服务端发送请请求内容：state
* 发布/angerforce话题
*************************************/
#include "ros/ros.h"
#include "port_service/state.h"

#include<cstdio>
#include<iostream>
#include<string>
#include<sstream>
using namespace std;

#define rBUFFERSIZE 8 //接收串口缓存长度
unsigned char r_buffer[rBUFFERSIZE];//接收缓存
unsigned char anger_buf[2];
unsigned char force_buf[2];
Float32 anger;
Float32 force;


// 16进制转10进制
unsigned int hex2int(const char* str)
{
    int size = strlen(str);
    unsigned int result = 0;
    for (int i = 0;i < size; i++)
    {
      	char chr = str[i];
		unsigned int value = 0;
		if (chr >= 'a' && chr <= 'f')
		{
			value = chr - 'a' + 10;
        }
        else if (chr >= 'A' && chr <= 'F')
		{
			value = chr - 'A' + 10;
		}
		else if (chr >= '0' && chr <= '9')
		{
			value = chr - '0';
		}
		result = result * 16 + value;
	}
	return result;
}

// CRC8校验
unsigned char CRC8(unsigned char *buffer)
{
    unsigned char ret=0;
    unsigned char csum;
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

int main(int argc, char** argv)
{
    // ROS初始化
    ros::init(argc, argv, "port_client");
    ros::NodeHandle nh;
    // 创建client并连接到server
    ros::service::waitForService("/port_server");
    ros::ServiceClient client = nh.serviceClient<port_service::state>("/port_server");
    // 创建发布anger和force的话题
    ros::Publisher angerforce = nh.advertise<std_msgs::Float32>("angerforce",1000);
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
            // 打包数据
            if(CRC8(r_buffer) != 0)
            {
                int i;
                for (i = 0; i < 2, i++);
                {
                    anger_buf[i] = r_buffer[2+i];
                    force_buf[i] = r_buffer[4+i];
                }
                anger = hex2int(anger_buf)/32767*180;
                force = hex2int(force_buf)/32767*180;
            }
        
            
            
            
            
            // 配置request数据
            port_service::state srv;
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

