#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "assignement_3/User.h"    // header of costumized message
#include <iostream>
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalID.h"
//#include "actionlib_msgs/GoalStatusArray.h"

int input;  // store the mode entred by the user 
//int status;

float x,y,linear_x,angular_z;

ros::Publisher pub_goal;
ros::Publisher pub_cancel;	//cancel the target
ros::Publisher pub_vel;	//publish a velocity to cmd_vel


/*void read_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
status= msg->status_list[0].status;
}*/

//subscriber callback to topic cmd_vel
void read_velocity(const geometry_msgs::Twist::ConstPtr& msg){
linear_x= msg->linear.x;
angular_z= msg->angular.z;
}

//subscriber callback of topic(user_topic) containing the data entred by the user
void user_input(const assignement_3::User::ConstPtr& msg)
{
 //printf("status = %d \n",status);
 input= msg->user;
 x= msg->x;
 y= msg->y;  

 if(input==1){
 printf("You are in Mode (1),Reaching a goal  \n");
 printf("you Can cancel by pressing '4' \n");
 move_base_msgs::MoveBaseActionGoal target;
 target.goal.target_pose.pose.position.x = x;
 target.goal.target_pose.pose.position.y = y;
 target.goal.target_pose.header.frame_id = "map";
 target.header.frame_id = "map";
 target.goal.target_pose.pose.orientation.w = 1.0;
 pub_goal.publish(target); 
 }
 
 if(input==2){
 printf("You are in Mode (2), NOT ASSISTED Driving  \n");
 // publish to this topic an empty msg to cancel the goal 
 actionlib_msgs::GoalID cancel;
 pub_cancel.publish(cancel);
 }
 if(input==3){printf("You are in Mode (3), ASSISTED Driving \n");}
 if(input==4){
 printf("canceling the target \n");
 // publish to this topic an empty msg to cancel the goal 
 actionlib_msgs::GoalID cancel;
 pub_cancel.publish(cancel);
 }
}

//subscriber to scan topic
void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //if(input ==1){ printf("status = %d \n",status);}
// the if statement used to cancel the target if user enters Mode2
if(input==2){ 
	actionlib_msgs::GoalID cancel;
 	pub_cancel.publish(cancel);
 	}

// these variables are used to declare the initial index of each range  
int i1=300;
int i2=420;
int i3=0;
//the min is used to returne the most closest obstacl in eache range 
float min1,min2,min3;
min1=msg->ranges[300];
min2=msg->ranges[660];
min3=msg->ranges[0];

//the for loops used to calculate the minimum value 
for( i1 =300; i1 < 420; i1++){
//front
         if(msg->ranges[i1] < min1){    
            min1 = msg->ranges[i1]; } 
        }


for( i2 =660; i2 < 720; i2++){
 //left
         if(msg->ranges[i2] < min2){
            min2 = msg->ranges[i2];} 
        }
for( i3 =0; i3 < 60; i3++){
 //right
         if(msg->ranges[i3] < min3){
            min3 = msg->ranges[i3];} 
        }        
 
        
if(input==3){
    actionlib_msgs::GoalID cancel;
    pub_cancel.publish(cancel);
    //check if there is a close obstacle in the front and the user drive the robot forward
    //if yes publish 0 linear velocity
    if(min1<0.4 && linear_x>0.0 ){
        printf("you can not GO_FORWARD !!  \n");
        geometry_msgs::Twist my_vel;
        my_vel.linear.x = 0.0;
        pub_vel.publish(my_vel);
        }
    //check if there is a close abstacle on the right and the user turn the robot to the right
    //if yes publish 0 angular velocity
    if(min3<0.4 && angular_z<0.0 ){
        printf("you cant turn CLOCK_WISE !!  \n");
         geometry_msgs::Twist my_vel;
         my_vel.angular.z = 0.0;
         pub_vel.publish(my_vel);
 	}
 
    //check if there is a close abstacle on the left and the user turn the robot to the left
    //if yes publish 0 angular velocity
    if(min2<0.4 && angular_z>0.0 ){
        printf("you cant turn COUNTER_CLOCK_WISE !!  \n");
        geometry_msgs::Twist my_vel;
        my_vel.angular.z = 0.0;
        pub_vel.publish(my_vel);
 	}
   }
}


int main (int argc, char **argv)
{
ros::init(argc, argv, "pubsub");
ros::NodeHandle nh;
ros::Subscriber sub_scan = nh.subscribe("/scan", 10, robotCallback);
ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 10, read_velocity);
//ros::Subscriber sub3 = nh.subscribe("/move_base/status", 100000, read_status);
ros::Subscriber sub_user = nh.subscribe("user_topic", 10, user_input);
pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal> ("/move_base/goal", 100); 
pub_cancel = nh.advertise<actionlib_msgs::GoalID> ("/move_base/cancel", 100); 
pub_vel = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10); 
ros::spin();

return 0;
}
