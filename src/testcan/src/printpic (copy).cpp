#include "testcan/Frame.h"
#include "testcan/canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "iostream"
#include "fstream"
#include "string.h"
#include "testcan/IpPos.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;
double pic[6];
int IC;
float out_force;
float cur_pose_init;
float out_pose_init;
int cur_pose;
int out_pose;
int cur_record=0;
int out_record=0;
float out_theta;
float cur_theta;

class JointState{
public:
    JointState();
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void init();
    void Cur_PosFeedCallback(const testcan::IpPos::ConstPtr &can_msg);
    void Out_PosFeedCallback(const testcan::IpPos::ConstPtr &can_msg);
    void Out_ForceFeedCallback(const std_msgs::Float64MultiArray &can_msg);
    void Out_ICFeedCallback(const testcan::IpPos::ConstPtr &can_msg);

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
    ros::Subscriber ip_cur_pos_sub;
    ros::Subscriber ip_out_pos_sub;
    ros::Subscriber ip_out_force_sub;
    ros::Subscriber ip_out_IC_sub;
    ros::NodeHandle nh;
};

int main(int argc, char *argv[])
{
    printf("hello\n");
    ros::init(argc, argv, "printpic");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    JointState JointState1;

    JointState1.registerNodeHandle(nh);
    JointState1.registerPubSub();
    JointState1.init();
    ofstream outf;
    outf.open("/home/s/abc11.txt");
    while (ros::ok())
    {
        ros::spinOnce();

        out_theta=out_pose-out_pose_init;
        cur_theta=cur_pose-cur_pose_init;

        pic[0]=out_theta-cur_theta/160;//D
        pic[1]=out_theta;//FSO
        pic[2]=IC;//I
        pic[3]=out_force;//T
        pic[4]=cur_theta;//MOTOR
        ROS_INFO("current position of encoder is %d",IC);
        outf<<pic[0]<<"\t"<<"\t"<<pic[1]<<"\t"<<"\t"<<pic[2]<<"\t"<<"\t"<<pic[3]<<"\t"<<"\t"<<pic[4]<<endl;
        loop_rate.sleep();

    }
    outf.close();
    return 0;
}

JointState::JointState(){};

void JointState::registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh;
};

void JointState::registerPubSub(){
    jointFeedbackPub = nh.advertise<sensor_msgs::JointState>("/joint_states",8);
    ip_cur_pos_sub = nh.subscribe("cur_pos",5,&JointState::Cur_PosFeedCallback,this);
    ip_out_pos_sub = nh.subscribe("absolute_pos",5,&JointState::Out_PosFeedCallback,this);
    ip_out_force_sub = nh.subscribe("six_axis_force_1",5,&JointState::Out_ForceFeedCallback,this);
    ip_out_IC_sub = nh.subscribe("I_c",5,&JointState::Out_ICFeedCallback,this);
}

void JointState::init(){
    for(int i =0;i<7;i++)
        {
            jointFeedbackMsg.position.push_back(0);
            jointFeedbackMsg.name.push_back("joint"+std::to_string(i+1));
        }
}

//void JointState::PosFeedCallback(const testcan::Frame::ConstPtr &can_msg)

void JointState::Cur_PosFeedCallback(const testcan::IpPos::ConstPtr &can_msg){
    unsigned char data[8];
    int dev_id;
    if(cur_record==0)//打开设备
    {
        cur_pose=can_msg->pos;
        dev_id = can_msg->id - TPDO2;
        jointFeedbackMsg.position[dev_id-1] = 0;
        jointFeedbackMsg.header.stamp = ros::Time::now();
        cur_pose_init=cur_pose;
        cur_record=1;
    }else
    {
        cur_pose=can_msg->pos;
        dev_id = can_msg->id - TPDO2;
        jointFeedbackMsg.position[dev_id-1] = 0;
        jointFeedbackMsg.header.stamp = ros::Time::now();
    }

};

void JointState::Out_PosFeedCallback(const testcan::IpPos::ConstPtr &can_msg){
    unsigned char data[8];
    int dev_id;
    if(out_record==0)
    {
        out_pose=can_msg->pos;
        dev_id = can_msg->id - TPDO2;
        jointFeedbackMsg.position[dev_id-1] = 0;
        jointFeedbackMsg.header.stamp = ros::Time::now();
        out_pose_init=out_pose;
        out_record=1;
    }else
    {
        out_pose=can_msg->pos;
        dev_id = can_msg->id - TPDO2;
        jointFeedbackMsg.position[dev_id-1] = 0;
        jointFeedbackMsg.header.stamp = ros::Time::now();
    }

};

void JointState::Out_ForceFeedCallback(const std_msgs::Float64MultiArray &can_msg){
    unsigned char data[8];
    int dev_id;

//    for(int i = 0; i < 4; i++)
//        {
//                data[i] = can_msg->data[i];
//        }
    out_force=can_msg.data[5];

    //ROS_INFO("out_force of encoder is %f",out_force);

    //int absolute_pos = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
    //dev_id = can_msg.id - TPDO2;
//    jointFeedbackMsg.position[dev_id-1] = 0;
//    jointFeedbackMsg.header.stamp = ros::Time::now();
    //jointFeedbackPub.publish(jointFeedbackMsg);

};
void JointState::Out_ICFeedCallback(const testcan::IpPos::ConstPtr &can_msg){
    unsigned char data[8];
    int dev_id;

//    for(int i = 0; i < 4; i++)
//        {
//                data[i] = can_msg->data[i];
//        }
    IC=can_msg->pos;

    //ROS_INFO("out_force of encoder is %f",out_force);

    //int absolute_pos = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|data[0];
    //dev_id = can_msg.id - TPDO2;
//    jointFeedbackMsg.position[dev_id-1] = 0;
//    jointFeedbackMsg.header.stamp = ros::Time::now();
    //jointFeedbackPub.publish(jointFeedbackMsg);

};


