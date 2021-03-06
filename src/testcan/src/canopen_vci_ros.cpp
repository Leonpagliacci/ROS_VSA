#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <iostream>
// #include "controlcan.h"
#include "canopen_vci_ros.h"
// #include "ros/ros.h"
// #include "testcan/Frame.h"
#include "std_msgs/String.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"
// #define dev_id 0x07
 
// VCI_BOARD_INFO pInfo;//用来获取设备信息。1.ZLGCAN系列接口卡信息的数据类型。
int count=1,sendcount = 1,packed_num=0,test_motor_id;//数据列表中，用来存储列表序号。
testcan::Frame _can_receive, _can_receive2;
//int CANopenVCIROS::motor_init_pos[28];
int flagrecord[28];
bool enable_leg,enable_arm,enable_wheel,enable_AP_feedback,enable_RP_feedback,enable_I_feedback,enable_V_feedback,enable_wheelspeed_feedback,single_motor_test,two_motor;
VCI_CAN_OBJ psend[51];//2.定义CAN信息帧的数据类型。
// VCI_CAN_OBJ *psend = send;
// VCI_CAN_OBJ send[48];
// char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

// class CANopenVCIROS
// {
// public:
//     CANopenVCIROS();
//     virtual ~CANopenVCIROS();
//     ros::Publisher canrecieve_pub;
//     int send(int id, int CANx, int type, const char *pdata);
//     int PDO_init(int dev_id);
//     static testcan::Frame can_receive;
//     static void *receive_func(void* param); 
//     static void cansendCallback(const testcan::Frame::ConstPtr& canmsg);
//     template <typename sdo>
//         char* SDODataPack(sdo data, unsigned short index, char subindex,char (&sdoptr)[8]);
    
// private:
//     VCI_BOARD_INFO pInfo;//用来获取设备信息
//     ros::NodeHandle nh_;
//     // ros::Publisher canrecieve_pub;
//     ros::Subscriber cansend_sub;
//     /* data */
// };
CANopenVCIROS::CANopenVCIROS(){
    
        printf(">>this is hello !\r\n %d",init_structure_motor_pos[17]);


};
//析构函数
CANopenVCIROS::~CANopenVCIROS(){
        VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
        usleep(100000);//延时100ms。
        VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
        usleep(100000);//延时100ms。
        VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
};

void CANopenVCIROS::CanDevInit(){
        printf("hou \n");//指示程序已运行
        //printf(" V %04d",VCI_OpenDevice(VCI_USBCAN2,0,0));printf("\n");
        if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
        {
            printf(">>open deivce success!\n");//打开设备成功
        }else
        {
            //printf(">>open deivce error!\n");
            exit(1);
        }
        if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
        {
            printf(">>Get VCI_ReadBoardInfo success!\n");
            
            //printf(" %08X", pInfo.hw_Version);printf("\n");
            //printf(" %08X", pInfo.fw_Version);printf("\n");
            //printf(" %08X", pInfo.dr_Version);printf("\n");
            //printf(" %08X", pInfo.in_Version);printf("\n");
            //printf(" %08X", pInfo.irq_Num);printf("\n");
            //printf(" %08X", pInfo.can_Num);printf("\n");
            printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
            printf("%c", pInfo.str_Serial_Num[1]);
            printf("%c", pInfo.str_Serial_Num[2]);
            printf("%c", pInfo.str_Serial_Num[3]);
            printf("%c", pInfo.str_Serial_Num[4]);
            printf("%c", pInfo.str_Serial_Num[5]);
            printf("%c", pInfo.str_Serial_Num[6]);
            printf("%c", pInfo.str_Serial_Num[7]);
            printf("%c", pInfo.str_Serial_Num[8]);
            printf("%c", pInfo.str_Serial_Num[9]);
            printf("%c", pInfo.str_Serial_Num[10]);
            printf("%c", pInfo.str_Serial_Num[11]);
            printf("%c", pInfo.str_Serial_Num[12]);
            printf("%c", pInfo.str_Serial_Num[13]);
            printf("%c", pInfo.str_Serial_Num[14]);
            printf("%c", pInfo.str_Serial_Num[15]);
            printf("%c", pInfo.str_Serial_Num[16]);
            printf("%c", pInfo.str_Serial_Num[17]);
            printf("%c", pInfo.str_Serial_Num[18]);
            printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

            printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
            printf("%c", pInfo.str_hw_Type[1]);
            printf("%c", pInfo.str_hw_Type[2]);
            printf("%c", pInfo.str_hw_Type[3]);
            printf("%c", pInfo.str_hw_Type[4]);
            printf("%c", pInfo.str_hw_Type[5]);
            printf("%c", pInfo.str_hw_Type[6]);
            printf("%c", pInfo.str_hw_Type[7]);
            printf("%c", pInfo.str_hw_Type[8]);
            printf("%c", pInfo.str_hw_Type[9]);printf("\n");	
        }else
        {
            printf(">>Get VCI_ReadBoardInfo error!\n");
            exit(1);
        }

        //初始化参数，严格参数二次开发函数库说明书。
        VCI_INIT_CONFIG config;
        config.AccCode=0;
        config.AccMask=0xFFFFFFFF;
        config.Filter=1;//接收所有帧
        config.Timing0=0x00;/*波特率125 Kbps  0x03  0x1C*/
        config.Timing1=0x14;
        config.Mode=0;//正常模式		
        /* 设置结构体参数的Timing0和Timing1可以设置CAN的波特率，常见的波特率设置如下：
    CAN波特率         定时器0      定时器1
    5Kbps               0xBF        0xFF
    10Kbps              0x31        0x1C
    20Kbps              0x18        0x1C
    40Kbps              0x87        0xFF
    50Kbps              0x09        0x1C
    80Kbps              0x83        0Xff
    100Kbps             0x04        0x1C
    125Kbps             0x03        0x1C
    200Kbps             0x81        0xFA
    250Kbps             0x01        0x1C
    400Kbps             0x80        0xFA
    500Kbps             0x00        0x1C
    666Kbps             0x80        0xB6
    800Kbps             0x00        0x16
    1000Kbps            0x00        0x14*/
        
        if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
        {
            printf(">>Init CAN1 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);
        }

        if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
        {
            printf(">>Start CAN1 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);

        }

        if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
        {
            printf(">>Init can2 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);

        }
        if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
        {
            printf(">>Start can2 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);

        }
}

void CANopenVCIROS::rosnode(){
    ros::NodeHandle nh_;
    canrecieve_pub = nh_.advertise<testcan::Frame>("can_recieve", 16);
    canrecieve2_pub = nh_.advertise<testcan::Frame>("can_recieve2", 1);
    cansend_sub = nh_.subscribe("can_send", 1, &CANopenVCIROS::cansendCallback);
    ip_pos_sub = nh_.subscribe("ip_pos",16,&CANopenVCIROS::PosCmdCallback);
    cmd_vel_sub = nh_.subscribe("cmd_vel",1,&CANopenVCIROS::veltowheelCallback);
    nh_.param("enable_leg",enable_leg,false);
    nh_.param("enable_arm",enable_arm,false);
    nh_.param("enable_wheel",enable_wheel,false);
    nh_.param("enable_AP_feedback",enable_AP_feedback,false);
    nh_.param("enable_RP_feedback",enable_RP_feedback,false);
    nh_.param("enable_I_feedback",enable_I_feedback,false);
    nh_.param("enable_V_feedback",enable_V_feedback,false);
    nh_.param("enable_wheelspeed_feedback",enable_wheelspeed_feedback,false);
    nh_.param("single_motor_test",single_motor_test,false);
    nh_.param("two_motor",two_motor,true);
    nh_.param("test_motor_id",test_motor_id,4);
    nh_.getParam("test_motor_id",test_motor_id);
    std::cout<<"getparam "<<test_motor_id<<std::endl;
};

int CANopenVCIROS::PDO_init(int dev_id){
    char IP_Mode_init[56][8] = {{0x40, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x40, 0x18, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                {0x2B, 0x0C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x2F, 0x0D, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x80},
                                {0x23, 0x14, 0x10, 0x00, 0x86, 0x00, 0x00, 0x00},
                                {0x23, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00},
                                {0x2B, 0x17, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x80},//RPDO1无效
                                {0x2F, 0x00, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},//RPDO1通信参数
                                {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},//清除RPDO映射对象
                                {0x23, 0x00, 0x16, 0x01, 0x08, 0x00, 0x60, 0x60},//RPDO1映射对象6060(模式选择)
                                {0x2F, 0x00, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00},//RPDO1映射对象一个
                                {0x23, 0x00, 0x14, 0x01, dev_id, 0x02, 0x00, 0x00},//RPDO1有效
                                {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x80},//RPDO2无效
                                {0x2F, 0x01, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},//RPDO2通信参数
                                {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60},//RPDO2映射对象6040(控制字)
                                {0x23, 0x01, 0x16, 0x02, 0x20, 0x00, 0x7A, 0x60},//RPDO2映射对象607A(目标位置)
                                {0x2F, 0x01, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x14, 0x01, dev_id, 0x03, 0x00, 0x00},//RPOD2有效
                                {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x80},//RPDO3无效
                                {0x2F, 0x02, 0x14, 0x02, 0xFF, 0x00, 0x00, 0x00},
                                {0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x02, 0x16, 0x01, 0x10, 0x01, 0xC1, 0x60},//RPDO3映射对象60C1(位置低位)
                                {0x23, 0x02, 0x16, 0x02, 0x10, 0x02, 0xC1, 0x60},//RPDO3映射对象60C1(位置高位)
                                {0x2F, 0x02, 0x16, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x02, 0x14, 0x01, dev_id, 0x04, 0x00, 0x00},
                                {0x23, 0x03, 0x14, 0x01, dev_id, 0x05, 0x00, 0x80},
                                {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x80},//TPDO1无效(可修改参数)
                                {0x2F, 0x00, 0x18, 0x02, 0x00, 0x00, 0x00, 0x00},//TPDO1通信参数
                                {0x2B, 0x00, 0x18, 0x03, 0x64, 0x00, 0x00, 0x00},//TPDO1禁止时间
                                {0x2B, 0x00, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},//TPOD1事件计时器
                                {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},
                                {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60},//TPDO1映射对象6041
                                {0x2F, 0x00, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00},//TPDO1映射对象个数1
                                {0x23, 0x00, 0x18, 0x01, 0x80+dev_id, 0x01, 0x00, 0x00},//TPDO1有效
                                {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x80},//TPDO2无效(可修改参数)
                                {0x2F, 0x01, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00},//00-FF(youxiao)
                                {0x2B, 0x01, 0x18, 0x03, 0x64, 0x00, 0x00, 0x00},//time
                                {0x2B, 0x01, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},
                                {0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},//清除TPDO映射对象
                                {0x23, 0x01, 0x1A, 0x01, 0x20, 0x00, 0xA0, 0x20},//TPDO2映射对象607A(juedui反馈)
                                {0x23, 0x01, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60},//TPDO2映射对象6064(位置反馈)
                                {0x2F, 0x01, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x01, 0x18, 0x01, 0x80+dev_id, 0x02, 0x00, 0x00},//TPDO2有效
                                {0x23, 0x02, 0x18, 0x01, 0x80+dev_id, 0x03, 0x00, 0x80},//TPDO3无效(可修改参数)
                                {0x2F, 0x02, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00},//00-FF(youxiao)
                                //{0x2B, 0x02, 0x18, 0x03, 0x64, 0x00, 0x00, 0x00},//time
                                {0x2B, 0x02, 0x18, 0x03, 0xC8, 0x00, 0x00, 0x00},//time
                                {0x2B, 0x02, 0x18, 0x05, 0x00, 0x00, 0x00, 0x00},
                                {0x2F, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00},//清除TPDO映射对象
                                {0x23, 0x02, 0x1A, 0x01, 0x20, 0x00, 0x69, 0x60},//TPDO3映射对象6064(速度反馈)

                                {0x23, 0x02, 0x1A, 0x02, 0x10, 0x00, 0x78, 0x60},//TPDO3映射对象6078(dianliu反馈)

                                {0x2F, 0x02, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00},
                                {0x23, 0x02, 0x18, 0x01, 0x80+dev_id, 0x03, 0x00, 0x00},
                                {0x23, 0x03, 0x18, 0x01, 0x80+dev_id, 0x04, 0x00, 0x80}};

        for (int i = 0;i < 56;i++){
            printf("OK %u \t",i+1);
            send(dev_id,CAN1,WriteSDO,IP_Mode_init[i],1);///48个包48次
            usleep(10000);

        }
    char modeop[8] = {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    char TPDO_enable[5][8] = {{0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                    {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                    {0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                    {0x1F,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00},
                    {0x1F,0x00,0x00,0x00,0xF9,0x15,0x00,0x00}};
    char start[8] = {0x01,dev_id,0x00,0x00,0x00,0x00,0x00,0x00};
    send(0,CAN1,0x000,start,1);
    usleep(10000);
    send(dev_id,CAN1,RPDO1,modeop,1);
    usleep(10000);
    send(dev_id,CAN1,RPDO2,TPDO_enable[4],3);
    // send(dev_id,CAN1,RPDO2,TPDO_enable[0],1);
    // send(dev_id,CAN1,RPDO2,TPDO_enable[1],1);
    // send(dev_id,CAN1,RPDO2,TPDO_enable[2],1);
}

void CANopenVCIROS::cansendCallback(const testcan::Frame::ConstPtr& canmsg){
    VCI_CAN_OBJ send[1];
	send[0].ID = canmsg->id;
	send[0].SendType=canmsg->is_error;
	send[0].RemoteFlag=canmsg->is_rtr;
	send[0].ExternFlag=canmsg->is_extended;
	send[0].DataLen=canmsg->dlc;
	
	int i=0;
	for(i = 0; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = canmsg->data[i];
	}
    if(VCI_Transmit(VCI_USBCAN2, 0, CAN1, send, 1) == 1)
		{
			printf("Index:%04d  ",count);count++;
			printf("CAN1 TX ID:0x%08X",send[0].ID);
			if(send[0].ExternFlag==0) printf(" Standard ");
			if(send[0].ExternFlag==1) printf(" Extend   ");
			if(send[0].RemoteFlag==0) printf(" Data   ");
			if(send[0].RemoteFlag==1) printf(" Remote ");
			printf("DLC:0x%02X",send[0].DataLen);
			printf(" data:0x");

			for(i=0;i<send[0].DataLen;i++)
			{
				printf(" %02X",send[0].Data[i]);
			}

			printf("\n");
			send[0].ID+=1;
		}
}

int CANopenVCIROS::startRecieveCanThread(){
    canRecieveThread_ = boost::thread(boost::bind(&CANopenVCIROS::canRecieveThread, this));
}

void CANopenVCIROS::canRecieveThread(){
    int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;

    int ind=0;
	
	while(ros::ok())
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
//            printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);
//            printf(" data:0x");
//            printf(" %02X", rec[j].Data[i]);
//            printf("\n");
			for(j=0;j<reclen;j++)
			{
                _can_receive.id = rec[j].ID;
                _can_receive.dlc = rec[j].DataLen;
                if(_can_receive.id > 0x180 && _can_receive.id < 0x18F){
                    printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);
                    printf(" data:0x");
                }
				for(i = 0; i < rec[j].DataLen; i++)
				{
                    if(_can_receive.id > 0x180 && _can_receive.id < 0x18F){
                        printf(" %02X", rec[j].Data[i]);
                    }
                    _can_receive.data[i] = rec[j].Data[i];
				}
                if(_can_receive.id > 0x180 && _can_receive.id < 0x18F){  //PDO1shuju
                    printf("\n");
                    }
                if(_can_receive.id > 0x280 && _can_receive.id < 0x480){  //PDO23shuju
                    canrecieve_pub.publish(_can_receive);
                }
                if (_can_receive.data[1]==0x64){
                    GetInitPos(_can_receive);//which encoder??
                }

			}
		}
        if((reclen=VCI_Receive(VCI_USBCAN2,0,1,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;//序号递增
				printf("CAN%d RX ID:0x%08X", 2, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
                _can_receive2.id = rec[j].ID;
                _can_receive2.dlc = rec[j].DataLen;
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
                    _can_receive2.data[i] = rec[j].Data[i];
				}
                // if (_can_receive2.data[0]==0x43){
                //     GetInitPos(_can_receive);
                // }
                // canrecieve_pub.publish(can_receive);
				//printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");
                // printf("%d\n",reclen);
			}
		}
	//  ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	printf("run thread exit\n");//退出接收线程
}

void *CANopenVCIROS::receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
    // testcan::Frame can_receive;
    // ros::Publisher canrecieve_pub;
    
	int *run=(int*)param;//线程启动，退出控制。
    int ind=0;
	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;//序号递增
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
                _can_receive.id = rec[j].ID;
                _can_receive.dlc = rec[j].DataLen;
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
                    _can_receive.data[i] = rec[j].Data[i];
				}
                if (_can_receive.data[1]==0x64){
                    GetInitPos(_can_receive);
                }
                // canrecieve_pub.publish(can_receive);
				//printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");
                // printf("%d\n",reclen);
			}
		}
        if((reclen=VCI_Receive(VCI_USBCAN2,0,1,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;//序号递增
				printf("CAN%d RX ID:0x%08X", 2, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
                _can_receive2.id = rec[j].ID;
                _can_receive2.dlc = rec[j].DataLen;
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
                    _can_receive2.data[i] = rec[j].Data[i];
				}
                // if (_can_receive2.data[0]==0x43){
                //     GetInitPos(_can_receive);
                // }
                // canrecieve_pub.publish(can_receive);
				//printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");
                // printf("%d\n",reclen);
			}
		}
	//  ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

void CANopenVCIROS::GetInitPos(testcan::Frame can_receive){
    char initial_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    int dev_id = can_receive.id - ReadSDO;
    if ((can_receive.id > ReadSDO)&&(flagrecord[dev_id] == 0)){
        unsigned char data[8];
        for(int i = 0; i < 8; i++)
        { 
            data[i] = can_receive.data[i];
        } 
        int init_pos_origin = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4]; 
        int init_pos = init_pos_origin + init_structure_motor_pos[dev_id-1];


        double init_pos_degree = init_pos*360/2129920;//20位编码器，减速比100
        if (dev_id ==1 || dev_id == 2 || dev_id == 8 || dev_id == 14){
            init_pos_degree = -init_pos_degree;
        }
        else{
            init_pos_degree = 360 - init_pos_degree;
        }

        //motor_init_pos[dev_id] = init_pos; 
        printf("INIT pos of motor %d is %d，equals to %lf degree.\n",dev_id,init_pos,init_pos_degree);
        flagrecord[dev_id] = 1;
    
        initial_pos[0] = (init_pos_origin<<24)>>24;
        initial_pos[1] = (init_pos_origin<<16)>>24;
        initial_pos[2] = (init_pos_origin<<8)>>24;
        initial_pos[3] = init_pos_origin>>24;
//        send(dev_id,CAN1,RPDO3,initial_pos,1);
    }
}

template <typename sdo>
char* CANopenVCIROS::SDODataPack(sdo data, unsigned short index, char subindex,char (&sdoptr)[8]){
    SDOData sdodata;
    char sdoarray[8];
    int i;
    size_t len = sizeof(data);
    sdodata.cmd = 0x23 + 4 * len - 4;
    sdodata.index = index;
    sdodata.subindex = subindex;
    switch (sdodata.cmd)
    {
        case 0x2f:
            /* 1字节数据 */
            sdodata.data[0] = data;
            sdodata.data[1] = 0x00;
            sdodata.data[2] = 0x00;
            sdodata.data[3] = 0x00;
            break;
        case 0x2b:
            sdodata.data[0] = (data<<8)>>8;
            sdodata.data[1] = data>>8;
            sdodata.data[2] = 0x00;
            sdodata.data[3] = 0x00;
            break;
        case 0x27:
            sdodata.data[0] = (data<<16)>>16;
            sdodata.data[1] = (data<<8)>>16;
            sdodata.data[2] = data>>16;
            sdodata.data[3] = 0x00;
            break;
        case 0x23:
            sdodata.data[0] = (data<<24)>>24;
            sdodata.data[1] = (data<<16)>>24;
            sdodata.data[2] = (data<<8)>>24;
            sdodata.data[3] = data>>24;
            break;
    }
    sdoarray[0] = sdodata.cmd;
	sdoarray[1] = (sdodata.index<<8)>>8;
	sdoarray[2] =  sdodata.index>>8;
	sdoarray[3] = sdodata.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdoarray[i] =  sdodata.data[i-4];
	}
    for (i = 0 ; i < 8 ; i++) {
		sdoptr[i] =  sdoarray[i];
	}
    // return sdoarray;
    

}

int CANopenVCIROS::send(int id, int CANx, int type, const char *pdata, unsigned int frame_num){
    //需要发送的帧，结构体设置
    SDOData sdodata;
    if (type==V||type==GV||type==ENA||type==DIS){
        type = 0;
        // pdata[1] = id;
    }
    // VCI_CAN_OBJ psend[48];
    if(packed_num<frame_num){
        // printf("pack data,pack num = %d\n",packed_num);
        psend[packed_num].ID = type + id;
        psend[packed_num].SendType=0;
        psend[packed_num].RemoteFlag=0;
        psend[packed_num].ExternFlag=0;
        psend[packed_num].DataLen=8;
	
        int i=0;
        for(i = 0; i < psend[packed_num].DataLen; i++)
        {
            psend[packed_num].Data[i] = pdata[i];
        }
        packed_num++;
    }
    // printf("send once, packed num = %d, frame num = %d\n",packed_num, frame_num);
    if(packed_num==frame_num){
        packed_num = 0;
        // printf("send once, packed num = %d\n",packed_num);
    	if(VCI_Transmit(VCI_USBCAN2, 0, CANx, psend, frame_num) >= 1)
		{
			printf("Index:%04d  ",count);
            count++;
			printf("CAN%d TX ID:0x%08X",CANx+1, psend[0].ID);
			if(psend[0].ExternFlag==0) printf(" Standard ");
			if(psend[0].ExternFlag==1) printf(" Extend   ");
			if(psend[0].RemoteFlag==0) printf(" Data   ");
			if(psend[0].RemoteFlag==1) printf(" Remote ");
			printf("DLC:0x%02X",psend[0].DataLen);
			printf(" data:0x");

			for(int i=0;i<psend[0].DataLen;i++)
			{
				printf(" %02X",psend[0].Data[i]);
			}

			printf("\n");
			psend[0].ID+=1;
		}
    }

}

// char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void CANopenVCIROS::PosCmdCallback(const testcan::IpPos::ConstPtr &cmd){
    if(single_motor_test)
    {
        double start = ros::Time::now().toNSec();
        ROS_INFO("recieve time is : %f",start);
    }
    char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    double data_receive_print = cmd->pos;
    double data_receive;
    //调整各个关节运动的正方向
    int dev_id = cmd->id;
    if (dev_id == 1 || dev_id == 2 ){
        data_receive = data_receive_print;
     }

    int data=0;

    if(dev_id ==1 || dev_id == 2)
    {
      data = int(data_receive*8192*50/360);//xiugai
    }
    

    //int data = int(data_receive*10485.76/0.036);//ningjing

   //int data = int(data_receive*4096*520/360);//xiugai


    //int data = int(data_receive*10485.76/0.036);//xiugai
    //data = data - init_structure_motor_pos[dev_id-1];
    printf("send pos is %lf",data_receive_print);

    ip_pos[0] = (data<<24)>>24;    //就是把前24位清零
    ip_pos[1] = (data<<16)>>24;    //把前16位和后8位清零；数据放在最右侧
    ip_pos[2] = (data<<8)>>24;      //把前8位和后16位清零；数据放在最右侧
    ip_pos[3] = data>>24;
    
    send(dev_id,CAN1,RPDO3,ip_pos,1);


    if(single_motor_test)
    {
        double end = ros::Time::now().toNSec();
        ROS_INFO("sended time is : %f",end);
    }
    printf("send %d times\n",sendcount);
    sendcount++;
}

//void CANopenVCIROS::PosCmdCallback(const testcan::IpPos::ConstPtr &cmd){

//    char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//    double data_receive = cmd->pos;

//    //调整各个关节运动的正方向
//    int dev_id = cmd->id;
//    //int data = int(data_receive*10485.76/0.036);//ningjing
//    int data = int(data_receive);//xiugai

//    ip_pos[0] = (data<<24)>>24;
//    ip_pos[1] = (data<<16)>>24;
//    ip_pos[2] = (data<<8)>>24;
//    ip_pos[3] = data>>24;

//    send(dev_id,CAN1,RPDO3,ip_pos,1);

//    sendcount++;
//}

void CANopenVCIROS::veltowheelCallback(const geometry_msgs::Twist &vel){
        char Can_A[8] = {0x08,0x11,V,0x00,0x00,0x00,0x00,0x00};
        char Can_B[8] = {0x08,0x22,V,0x00,0x00,0x00,0x00,0x00};
        char Can_C[8] = {0x08,0x33,V,0x00,0x00,0x00,0x00,0x00};
        char Can_D[8] = {0x08,0x44,V,0x00,0x00,0x00,0x00,0x00};
        int Target_A, Target_B, Target_C, Target_D;
        float Vx, Vy, Vz;
        Vx = vel.linear.x;
        Vy = vel.linear.y;
        Vz = vel.angular.z;
        Target_A   = 14*30*(Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER))/R_wheel/PI;
        Target_B   = -14*30*(Vx-Vy-Vz*(a_PARAMETER+b_PARAMETER))/R_wheel/PI;
	    Target_C   = -14*30*(Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER))/R_wheel/PI;
		Target_D   = 14*30*(Vx-Vy+Vz*(a_PARAMETER+b_PARAMETER))/R_wheel/PI;
        int2char(Target_A,Can_A);
        int2char(Target_B,Can_B);
        int2char(Target_C,Can_C);
        int2char(Target_D,Can_D);
        send(0x11,CAN2,V,Can_A,1);
        send(0x22,CAN2,V,Can_B,1);
        send(0x33,CAN2,V,Can_C,1);
        send(0x44,CAN2,V,Can_D,1);
        printf("velocity cmd is Vx = %f\t Vy = %f\t W = %f\t", Vx,Vy,Vz);
        printf("wheel velocity cmd is A = %d\t B = %d\t C = %d\t D = %d\t", Target_A, Target_B, Target_C, Target_D);
}
void CANopenVCIROS::int2char(int dat,char a[])
{
	unsigned int data;
	data=(unsigned int)dat;
	a[7]=data>>24;
	a[6]=data>>16;
	a[5]=data>>8;
	a[4]=data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "CANopen_ROS_node");
    CANopenVCIROS CANopenVCIROS1;   //实例化一个类
    printf("qian");
    CANopenVCIROS1.CanDevInit();
    CANopenVCIROS1.rosnode();
    CANopenVCIROS1.startRecieveCanThread();
    ros::NodeHandle nh;
    int wheel_id =0x00;
    // ros::NodeHandle nh;
    // ros::Subscriber cansend_sub = nh.subscribe("can_send", 1, CANopenVCIROS1.cansendCallback);
    // CANopenVCIROS CANopenVCIROS1;
    //启动接收线程
    //char sdoAP[8]={0x40,0xA0,0x20,0x00,0x00,0x00,0x00,0x00};//现在程序卡住，之前能动，但是显示为0
    char sdoAP[8]={0x40,0x7A,0x60,0x00,0x00,0x00,0x00,0x00};                                                                                                                                                                                                                                                                                                            
    char sdoRP[8]={0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00};
    char sdoI[8]={0x40,0x78,0x60,0x00,0x00,0x00,0x00,0x00};
    char sdoV[8]={0x40,0x69,0x60,0x00,0x00,0x00,0x00,0x00};
    char sdoStatus[8]={0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00};
    char sdoMode[8]={0x40,0x61,0x60,0x00,0x00,0x00,0x00,0x00};
    char sdotest[8]={0x40,0x3F,0x60,0x00,0x00,0x00,0x00,0x00};
    char sdoclear[8]={0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00};
    char sdo_enable[8]={0x2B,0x40,0x60,0x00,0x3F,0x00,0x00,0x00};
    // char initial_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    char Enable11[8]={0x08,0x11,ENA,0x00,0x00,0x00,0x00,0x00};
    char Enable22[8]={0x08,0x22,ENA,0x00,0x00,0x00,0x00,0x00};
    char Enable33[8]={0x08,0x33,ENA,0x00,0x00,0x00,0x00,0x00};
    char Enable44[8]={0x08,0x44,ENA,0x00,0x00,0x00,0x00,0x00};
    char DisEnable11[8]={0x08,0x11,DIS,0x00,0x00,0x00,0x00,0x00};
    char DisEnable22[8]={0x08,0x22,DIS,0x00,0x00,0x00,0x00,0x00};
    char DisEnable33[8]={0x08,0x33,DIS,0x00,0x00,0x00,0x00,0x00};
    char DisEnable44[8]={0x08,0x44,DIS,0x00,0x00,0x00,0x00,0x00};
    char GetVel[8]={0x08,0x11,GV,0x00,0x00,0x00,0x00,0x00};
    char GetVel11[8]={0x08,0x11,GV,0x00,0x00,0x00,0x00,0x00};
    char GetVel22[8]={0x08,0x22,GV,0x00,0x00,0x00,0x00,0x00};
    char GetVel33[8]={0x08,0x33,GV,0x00,0x00,0x00,0x00,0x00};
    char GetVel44[8]={0x08,0x44,GV,0x00,0x00,0x00,0x00,0x00};
    int m_run0=1;
	// pthread_t threadid;
	// int ret;
	// ret=pthread_create(&threadid,NULL,&CANopenVCIROS1.receive_func,&m_run0);
   
    if(single_motor_test){
        CANopenVCIROS1.PDO_init(test_motor_id);
        CANopenVCIROS1.send(test_motor_id,CAN1,WriteSDO,sdoAP,1);
        CANopenVCIROS1.send(test_motor_id,CAN1,WriteSDO,sdoRP,1);

    }
    if(two_motor){
        CANopenVCIROS1.PDO_init(1);
        CANopenVCIROS1.PDO_init(2);
        // CANopenVCIROS1.PDO_init(4);
        // CANopenVCIROS1.PDO_init(5);
        // CANopenVCIROS1.PDO_init(6);

        CANopenVCIROS1.send(1,CAN1,WriteSDO,sdoAP,1);
        CANopenVCIROS1.send(2,CAN1,WriteSDO,sdoAP,1);
        // CANopenVCIROS1.send(4,CAN1,WriteSDO,sdoAP,1);
        // CANopenVCIROS1.send(5,CAN1,WriteSDO,sdoAP,1);
        // CANopenVCIROS1.send(6,CAN1,WriteSDO,sdoAP,1);
        
        CANopenVCIROS1.send(1,CAN1,WriteSDO,sdoRP,1);
        CANopenVCIROS1.send(2,CAN1,WriteSDO,sdoRP,1);
        // CANopenVCIROS1.send(4,CAN1,WriteSDO,sdoRP,1);
        // CANopenVCIROS1.send(5,CAN1,WriteSDO,sdoRP,1);
        // CANopenVCIROS1.send(6,CAN1,WriteSDO,sdoRP,1);
    }

    ros::Rate loop_rate(100);
    
    usleep(1000000);

    // CANopenVCIROS1.send(5,CAN1,WriteSDO,sdoclear,1);
    // CANopenVCIROS1.send(6,CAN1,WriteSDO,sdoclear,1);

    // CANopenVCIROS1.send(5,CAN1,WriteSDO,sdo_enable,1);
    // CANopenVCIROS1.send(6,CAN1,WriteSDO,sdoclear,1);

 	while (ros::ok())
	{
        /*
        *通过SDO查询关节相对位置,绝对位置,电流,速度
        */
        // CANopenVCIROS1.send(0x06,CAN1,WriteSDO,sdoStatus,1);
        // CANopenVCIROS1.send(0x06,CAN1,WriteSDO,sdoMode,1);
        // CANopenVCIROS1.send(0x06,CAN1,WriteSDO,sdotest,1);

//        if(enable_AP_feedback)
//            CANopenVCIROS1.send(test_motor_id,CAN1,WriteSDO,sdoAP,1);
//        if(enable_RP_feedback)
//            CANopenVCIROS1.send(test_motor_id,CAN1,WriteSDO,sdoRP,1);
//        if(enable_I_feedback)
//            CANopenVCIROS1.send(test_motor_id,CAN1,WriteSDO,sdoI,1);
//        if(enable_V_feedback)
//            CANopenVCIROS1.send(test_motor_id,CAN1,WriteSDO,sdoV,1);

		// CANopenVCIROS1.canrecieve_pub.publish(_can_receive);//发布收到的CAN原始信息
        // CANopenVCIROS1.canrecieve2_pub.publish(_can_receive2);
        //  ROS_INFO("%s", "ok");
		ros::spinOnce();
        // CANopenVCIROS1.send(0,CAN1,WriteSDO,sdo1);
		loop_rate.sleep();
		// ++count;
	}
//        CANopenVCIROS1.send(0x11,CAN2,DIS,DisEnable11,4);
//        CANopenVCIROS1.send(0x22,CAN2,DIS,DisEnable22,4);
//        CANopenVCIROS1.send(0x33,CAN2,DIS,DisEnable33,4);
//        CANopenVCIROS1.send(0x44,CAN2,DIS,DisEnable44,4);

    // char stop[8] = {0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    // CANopenVCIROS1.send(3,CAN1,WriteSDO,stop,1);
    // usleep(1000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	m_run0=0;//线程关闭指令。
	//pthread_join(threadid,NULL);//等待线程关闭。

    return 0;
}
