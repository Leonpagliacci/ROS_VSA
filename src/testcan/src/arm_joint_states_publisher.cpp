#include "testcan/Frame.h"
#include "testcan/canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "iostream"
#include "fstream"
#include "string.h"

using namespace std;

class JointState{
public:
    JointState();
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void init();
    void PosFeedCallback(const testcan::Frame::ConstPtr &can_msg);
private:
    int init_structure_motor_pos[7] = {
            -20388978+1893261,
            4369067-2912711+10340124-11650844+1310718+52428800-17476267,
            -2330149+1747628+52428800,//2038898+52428800,
            -2038896+52428800,
            -1456355+1747625+52428800,//1456355+52428800,
            -16311182+26214400+291271+52428800,
            2912711+1747626,
            };
    double current_pos[7];
    sensor_msgs::JointState jointFeedbackMsg;
    ros::Publisher jointFeedbackPub;
    ros::Subscriber ip_pos_sub;
    ros::NodeHandle nh;
};

int main(int argc, char *argv[])
{
    printf("hello\n");
    ros::init(argc, argv, "arm_joint_states_publisher");
    ros::NodeHandle nh;
     
    JointState JointState1;

    JointState1.registerNodeHandle(nh);
    JointState1.registerPubSub();
    JointState1.init();
    
    ros::spin();

    return 0;
}

JointState::JointState(){};

void JointState::registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh;
};

void JointState::registerPubSub(){
    jointFeedbackPub = nh.advertise<sensor_msgs::JointState>("/joint_states",8);
    ip_pos_sub = nh.subscribe("can_recieve",40,&JointState::PosFeedCallback,this);
}

void JointState::init(){
    for(int i =0;i<7;i++)
        {
            jointFeedbackMsg.position.push_back(0);
            jointFeedbackMsg.name.push_back("joint"+std::to_string(i+1));
        }   
}

void JointState::PosFeedCallback(const testcan::Frame::ConstPtr &can_msg){
    unsigned char data[8];
    int dev_id;
    for(int i = 0; i < 4; i++)
	{
		data[i] = can_msg->data[i];
	} 

    int absolute_pos = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0]; 
    ROS_INFO("current position of encoder is %d",absolute_pos);
    dev_id = can_msg->id - TPDO2;

    int cur_pos = absolute_pos + init_structure_motor_pos[dev_id-1];
    //double current_pos_degree = cur_pos*0.036/10485.76;//20位编码器，减速比100
    double current_pos_degree = cur_pos*0.036/(209.7152*160);//20位编码器，减速比100
    // if (dev_id ==1 || dev_id == 7 || dev_id == 8 || dev_id == 14){
    //     current_pos_degree = -current_pos_degree;
    // }
    //  else{
    //      current_pos_degree = 360 - current_pos_degree;
    // }
    current_pos[dev_id-1] = current_pos_degree;
    jointFeedbackMsg.position[dev_id-1] = current_pos[dev_id-1];
    //ROS_INFO("current position of joint %d is %f",dev_id,current_pos[dev_id-1]);
    //OS_INFO("current position of joint %d is %02X %02X %02X %02X",dev_id,data[0],data[1],data[2],data[3]);

    jointFeedbackMsg.header.stamp = ros::Time::now();
    jointFeedbackPub.publish(jointFeedbackMsg);

};




