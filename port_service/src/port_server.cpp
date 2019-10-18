/************************************
*************服务器节点****************
**
**串口数据发送格式共8字节
** head  head  CMD  DF0  DF2  DF3 CRC8
** 0xA5  0x5A  short   short       u8
*************************************/
#include <ros/ros.h>
#include <port_service/state.h>
#include <std_msgs>

#define	sBUFFERSIZE	8//串口发送缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
bool pubCommand = false;

// 数据打包
void data_pack()
{
    s_buffer[1] = 0xa5;
    s_buffer[2] = 0x5a;
    int i；
    for(i=0;i<2;i++)
    {
        s_buffer[i] = r_buffer[2+i];//torque
        s_buffer[2+i] = r_buffer[4+i];//angle
        s_buffer[4] = s_buffer[0]^s_buffer[1]^s_buffer[2]^s_buffer[3];//crc
    } 
    s_buffer[7] = 
}

// 回调函数,收到client命令通过串口发送命令
void cmdCallback()
{
    // 创建话题
    ros::Publisher port_pub = nh.adbvertise<port_service::gb>("Tx",10);
    
    // 循环等待
    ROS_INFO('Waiting for command');
    
}
    
int main(int argc, char** argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "port_server");
    // 创建句柄
    ros::NodeHandle nh;
    // 创建服务/port_server，注册回调
    ros::ServiceServer service = nh.advertiseService("/port_server", cmdCallback);

    while(ros::ok())
    {
        //查看一次回调函数栈
        ros::spinOnce();
        if (pubCommand)
        {
            ser.write(s_buffer,sBUFFERSIZE;)
        }
    }

    // 循环频率
    ros::Rate loop_rate(10);

    return 0;
}