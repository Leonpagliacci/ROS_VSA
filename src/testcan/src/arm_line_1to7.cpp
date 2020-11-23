
#include "ros/ros.h"
#include "testcan/IpPos.h"  
#include "iostream"
#include "fstream"
#include "std_msgs/Float64.h"

#define postoangle 1440000.0/360 
#define big 104857600/360 //2498560.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0

using namespace std;
// std_msgs::Float64 output_Pos;
// std_msgs::Float64 input_Pos; 
// std_msgs::Float64 deflection;
// std_msgs::Float64 iniangel;
// std_msgs::Float64 stophere;
// std_msgs::Float64 iniangel2;
// float output_Pos;
// float input_Pos; 
// float deflection;
// float iniangel;
// float stophere;
// float iniangel2;

double output_Pos;
double input_Pos; 
double deflection;
double iniangel;
double stophere;
double iniangel2;
int index1=0;
int index2=0;
int flag=0;


testcan::IpPos ip_pos;
int sendangle(int id, double angle){
    ip_pos.id = id;
    ip_pos.pos = angle;
    return 0;
}

void abposCallback(const testcan::IpPos &ab_Pos){
    if(index1==0)
    {
        iniangel=abs(ab_Pos.angle);
        index1++;
        printf("initalOutoutAngle=%f\n",iniangel);
    }
    printf("deflection=%f\n",deflection);
    output_Pos=abs(ab_Pos.angle);
    // printf("output=%f\n",output_Pos);
}

void curposCallback(const testcan::IpPos &cur_Pos){
    if(index2==0)
    {
        iniangel2=abs(cur_Pos.angle);
        index2++;
        printf("initalMotorAngle=%f\n",iniangel2);
    }
    // printf("index2=%d\n",index2);
    stophere=cur_Pos.angle;
    input_Pos=abs(cur_Pos.angle);
}



int main(int argc, char *argv[])
{
    printf("hello\n");
    ros::init(argc, argv, "CommandGenerate");
    ros::NodeHandle nh_;
	printf("start\n");
    ros::Subscriber ab_pos_sub = nh_.subscribe("absolute_pos",32,abposCallback);
    ros::Subscriber cur_pos_sub = nh_.subscribe("cur_pos",32,curposCallback);
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1000);
    ros::Publisher def_pub = nh_.advertise<std_msgs::Float64>("def_angle",1000);
    
    usleep(500000);
    ros::Rate loop_rate(200);
    double j1,j2,j3,j4,j5,j6,j7;



    while (ros::ok())
	{
   //vmlinuz.old ifstream infile("/home/wjh/test.txt");
//infile>>j1>>j2>>j3>>j4>>j5>>j6>>j7;
//    while(!infile.eof())
//    {

//    }
    float destination=90;
    // float destination=45;

    
     sendangle(1,destination);  //POSITIVE
     pos_pub.publish(ip_pos);

    //  sendangle(6, 0);
    //  pos_pub.publish(ip_pos);

    sendangle(2,-destination);
    pos_pub.publish(ip_pos);
    // sendangle(3, j3);
    // pos_pub.publish(ip_pos);
    // sendangle(4, j4);
    // pos_pub.publish(ip_pos);
    // sendangle(5, j5);
    // pos_pub.publish(ip_pos);
    // sendangle(6, j6);
    // pos_pub.publish(ip_pos);   
    // sendangle(7, j7);
    // pos_pub.publish(ip_pos);




    

deflection=abs(abs(input_Pos-iniangel2)/50-abs(output_Pos-iniangel)); //计算一下关节的变形角度
std_msgs::Float64 def_msg;
def_msg.data=deflection;
def_pub.publish(def_msg);

if (abs(deflection)>=7)
{
    flag=1;
}




    ros::spinOnce();
    


    
//  while (flag==1)
//  {
//      printf("deflection=chaoguoleyuzhi%f\n",output_Pos);
//     //  printf("fasongzhi=%f",stophere);
//      float test1=0;
//   /*   sendangle(1,-stophere/50);  
//      pos_pub.publish(ip_pos);
//      sendangle(2,stophere/50);
//      pos_pub.publish(ip_pos);*/
//      sendangle(1,-test1);  
//      pos_pub.publish(ip_pos);
//      sendangle(2,test1);
//      pos_pub.publish(ip_pos);
//      ros::spinOnce();
//      if (abs(output_Pos-iniangel)<8)
//      {
//          flag=0;
//          break;
//      }    
//   }



     loop_rate.sleep();
   
	}


   
 /*       sendangle(1,input_Pos/50);
        pos_pub.publish(ip_pos);
        sendangle(2,-input_Pos/50);
        pos_pub.publish(ip_pos);*/

    
        

    
    printf("Bye\n");
    return 0;
}
