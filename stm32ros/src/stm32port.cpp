/****************************
 * ROS串口Demo
 * 通过串口接收数据
 * 通过串口发回接受的数据
 * **************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    // 创建句柄
    ros::NodeHandle n;
    // 创建serial类
    serial::Serial ser;
    // 串口名称
    ser.setPort("/dev//pts/20");
    // 波特率
    ser.setBaudrate(115200);
 
    try
    {
        // 打开串口
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    // 判断串口是否打开成功
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("/dev//pts/20 is opened.");
    }
    else
    {
        return -1;
    }
    // 10hz频率执行程序
    ros::Rate loop_rate(500);
    
    while(ros::ok())
    {
        // 获取缓冲区内的字节数
        size_t n = ser.available();
        if(n!=0)
        {
            uint8_t r_buffer[1024];
            // 读出数据
            n = ser.read(r_buffer, n);
            
            for(int i=0; i<n; i++)
            {
                // 16进制的方式打印到屏幕
                // std::cout << std::hex << (r_buffer[i] & 0xff) << " ";
                std::cout << (r_buffer[i] ) << " ";
            }
            std::cout << std::endl;
            // 把数据发送回去
            ser.write(r_buffer, n);
        }
        loop_rate.sleep();
    }
    return 0;
}