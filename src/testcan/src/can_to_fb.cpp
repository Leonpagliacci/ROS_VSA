#include "testcan/Frame.h"
#include "testcan/canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "ros/ros.h"
#include "iostream"
#include "fstream"
#define postoangle 1440000.0/360
#define big 104857600/360 //2498560.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0

using namespace std;

testcan::IpPos cur_pos_msg, absolute_pos_msg, vel_msg,Ic_msg, ip_pos_msg;
int last_motor_pos, last_out_pos, motor_pos, out_pos;
int flag = 0;
FILE *fp1;

// fp1 = fopen("catkin_ws/src/testcan/log/IandVel.txt", "w");
void PosFeedbackCallback(const testcan::Frame::ConstPtr &can_msg){
    unsigned char data[8];
    int dev_id;
    int Iom;
    Iom = 0;
    int vel;
    //printf("recieve!! \n");
    for(int i = 0; i < 8; i++)
    {
        data[i] = can_msg->data[i];
    }

    if (can_msg->id> 0x280 && can_msg->id < 0x380){

       // printf("recieve 111!! 0x%08X\n",data[0]);
        int cur_pos = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4]; //last 4 bete--cur
        int16_t I_raw = (data[5]<<8)|data[4]; //45wei

        vel = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0]; //qiansiwei--absolute
        dev_id = can_msg->id - TPDO2;
        //printf("recieve 111!! %I16d,%I16d\n",cur_pos,vel);


        cur_pos_msg.pos = cur_pos;
        cur_pos_msg.angle = 360*cur_pos/8192;
        cur_pos_msg.id = dev_id;

 
        absolute_pos_msg.id = dev_id;
        absolute_pos_msg.pos = -vel;
        if(absolute_pos_msg.id==2 )
        {
        absolute_pos_msg.angle = 360*vel/1048576;
        }

    }
    else if (can_msg->id> 0x380 && can_msg->id < 0x480){
//        printf("recieve 222!! \n");

        int cur_pos = (data[7]<<24)|(data[6]<<16)|(data[5]<<8)|data[4]; //last 4 bete--cur
        int16_t I_raw = (data[5]<<8)|data[4]; //45wei

        I_raw=9*I_raw;
        vel = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0]; //qiansiwei--absolute
        dev_id = can_msg->id - TPDO2;
        //printf("recieve 222!! 0x%08X\n",data[4]);
        //printf("recieve 222!! %d, 0x%08X\n",I_raw,I_raw);
//        printf("recieve 222!!  %04X,\n",I_raw);
        // printf("recieve 222!!  %04d,\n",I_raw);

        vel_msg.pos = cur_pos;
        if(vel_msg.id==1 || vel_msg.id==2 )
        {
          vel_msg.angle = 360*vel/2097152;
        }
         if(vel_msg.id==3 || vel_msg.id==4 )
        {
          vel_msg.angle = 360*vel/4096;
        }
        vel_msg.id = dev_id;

        Ic_msg.pos = I_raw;
        Ic_msg.angle = 0;
        Ic_msg.id = dev_id;


    }
    else{
        printf("recieve error!! \n");
    }
}


int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "can_to_fb");
    ros::NodeHandle nh_;
    //fp1 = fopen("catkin_ws/src/testcan/log/IandVel.txt", "w");
    //fprintf(fp1, "%s %s\n","I", "V");
    ros::Subscriber ip_pos_sub = nh_.subscribe("can_recieve",32,PosFeedbackCallback);
    ros::Publisher cur_pos_pub = nh_.advertise<testcan::IpPos>("cur_pos",1);
    ros::Publisher absolute_pos_pub = nh_.advertise<testcan::IpPos>("absolute_pos",1);
    ros::Publisher vel_pub = nh_.advertise<testcan::IpPos>("velocity",1);
    ros::Publisher ic_pub = nh_.advertise<testcan::IpPos>("I_c",1);
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1);
    ros::Publisher def_pub = nh_.advertise<std_msgs::Float64>("def_angle",1);
    // testcan::IpPos cur_pos_msg;
    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        /* code for loop body */
        cur_pos_pub.publish(cur_pos_msg);

        // printf("cur pos is %f\n",cur_pos_msg.angle);
        absolute_pos_pub.publish(absolute_pos_msg);
        // printf("absolute pos is %f\n",absolute_pos_msg.angle);
        vel_pub.publish(vel_msg);
        ic_pub.publish(Ic_msg);
        // if(Ic_msg.id==1)
        // {
        // printf("joint 1 i_c is %f\n",Ic_msg.pos);
        // }
        

          if(absolute_pos_msg.id==2)
        {
        ROS_INFO("motor angle %f", cur_pos_msg.angle);
        ROS_INFO("output angle %f \n", absolute_pos_msg.angle);
        }
      

        // if(vel_msg.id==1)
        // {
        //     printf("joint 4 vel is %f\n",vel_msg.angle );

        // }

        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;
}
