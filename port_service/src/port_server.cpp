/************************************
*************服务器节点****************
**串口数据接收格式共8字节
** head head Torque  angle cmd CRC
** 0xa5 0x5a   short   short     u8
*************************************
**串口数据发送格式共8字节
** head  head  CMD  DF0  DF2  DF3 CRC8
** 0xA5  0x5A  short   short       u8
*************************************/
#include <ros/ros.h>
#include <port_service/request.h>
#include <std_msgs>

#define	sBUFFERSIZE	8//串口发送缓存长度
#define	rBUFFERSIZE	8//串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

//联合体，用于浮点数与16进制的快速转换
typedef union{
    unsigned char cvalue[4];
    float fvalue;
}  float_union;

// CRC8校验，字节求异或
unsigned char crc8(unsigned char *buffer){
    unsigned char ret=0,csum;
		if((buffer[0]==0xa5) && (buffer[1]==0x5a)){
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6];
		//打印CRC校验码
		ROS_INFO("check sum:0x%02x",csum);
		if(csum == buffer[7]){
			ret = 1;
		}
		else 
		  ret =0;
	}
	return ret;
}

// 数据打包
void data_pack(){
    int i；
    for(i=0;i<2;i++){
        s_buffer[i] = r_buffer[2+i];//torque
        s_buffer[2+i] = r_buffer[4+i];//angle
        s_buffer[4] = s_buffer[0]^s_buffer[1]^s_buffer[2]^s_buffer[3];//crc
    } 
}

// 回调函数
void cmdCallback(){
    // 创建话题
    ros::Publisher port_pub = nh.adbvertise<port_service::gb>("Tx",10);
    
    // 创建串口类
    serial::Serial ser
    string port("/dev/ttyUSB0");//串口号
    unsigned long baud = 115200;//波特率

    // 循环等待
    ROS_INFO('Waiting for command');
    
    // 测试串口情况
    try{
        ser.setPort("/dev/pts/19");
        ser.setBaudrate(115200);
        ser.open();
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;}
        
    // 读取串口数据并发布话题
    

    
    
}
    
int main(int argc, char** argv){ 
    // ROS节点初始化
    ros::init(argc, argv, "port_server");
    // 创建句柄
    ros::NodeHandle nh;
    // 创建服务
    ros::ServiceServer service = nh.advertiseService("/port_server", cmdCallback);
    

    // 循环频率
    ros::Rate loop_rate(10);
    
    while(ros::OK){
        // 刷新回调函数队列
        ros::spinOnce()
        if(ser.available()){
            // 接收串口数据
            ROS_INFO_STREAM("Reading from serial port");
            ser.read(r_buffer,rBUFFERSIZE)
            // 打印串口数据
            int i;
            for(i=0;i<rBUFFERSIZE;i++)
            ROS_INFO("[0x%02x]",r_buffer[i]);
            ROS_INFO_STREAM("End Reading Serial Port");
            // CRC校验数据并存储数据
            if(crc8(r_buffer)!=0){
                int i;
                for(i=0;i<2;i++){
                   
                }
                s_buffer[4]=s_buffer[0]^s_buffer[1]^s_buffer[2]^s_buffer[3];
        }
        // 回调函数选择
        switch(pubCommand)
        {
            case pause
            {
	
                ser.write(s_buffer,sBUFFERSIZE);
                break；
            }
            case resume
            {
                ser.write(s_buffer,sBUFFERSIZE);
                break；
            }
            case restart
            {
                ser.write(s_buffer,sBUFFERSIZE);
                break；
            }
        }
    }