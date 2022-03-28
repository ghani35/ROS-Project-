/**
 * @file controller.cpp
 * @brief This is the controller node 
 * @author Bakour Abdelghani bakourabdelghani1999@gmail.com
 * @version 1.0
 * @date 25/12/2021
 * 
 * 
 * Publishers to:<BR> 
 *   /move_base/goal   /move_base/cancel  /cmd_vel  
 * 
 * Subscribers to:<BR>
 * 	 /user_topic   /scan   /cmd_vel   /move_base/status
 * 
 * Actions :<BR>
 *   /MoveBaseActionGoal   /GoalID   /GoalStatusArray
 * 
 * Description:  
 * Control the robot depending on the **mode** entred by the user.
 * If the user choose **mode1** he must set a goal point. The goal 
 * can be *canceled* by pressing "4" that takes you to **mode4**
 * and **mode3 and mode 4** are for driving the robot by the *keyboard*.
 *
 * 
 * @attention This node is run by the launch file **solution.launch**
 * @note In order for this node to **work** correctly,it is neccessary
 * @note to have other *packages* and *nodes* running concurrently.
 * @see user_interface.cpp
 **/


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "assignement_3/User.h"    
#include <iostream>
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"

int input;  ///< Stores the mode entred by the user 1,2,3 or 4.  
int status; ///< Stores the status of the goal (reachable or not), returned by the *read_status* callback funtion.
float x;///< Stores the x-axis coordinate of the goal of the form (x,y).
float y;///< Stores the y-axis coorfinate of the goal of the form (x,y).
float linear_x;///< Stores the linear velocity published to the robot.
float angular_z;///< Stores the angular velocity published to the tobot.
move_base_msgs::MoveBaseActionGoal target;///< Action used to set the goal point in case the robot is in mode1 
actionlib_msgs::GoalID cancel; ///< Action used to cancel the goal.

ros::Publisher pub_goal; ///< Global publisher to the topic */move_base/goal*, to publish the goal point.
ros::Publisher pub_cancel;	///< Global publisher to the topic */move_base/cancel*, to cancel the goal
ros::Publisher pub_vel;	///< Global publisher to the topic */cmd_vel*, to publish the velocity to the robot.


ros::Subscriber sub_scan; ///< Global subscriber to the topic */LaserScan*. 
ros::Subscriber sub_vel;///< Global subscriber to the topic */cmd_vel*, to retriev the current velocity of the robot.
ros::Subscriber sub_goal_status;///< Global subscriber to the topic */move_base/status*, to retriev the status of the goal.
ros::Subscriber sub_user;///< Global subscriber to the topic */user_topic*, to retriev the mode entred by the user.

/**
 * @brief Callback function of the subscriber sub_goal
 * @see sub_goal_status
 * @param msg actionlib message stores the goal status
 * @return nothing
 * @attention in order to know if the goal is not reacheable 
 * @attention the whole map should be discovered
 * 
 * @note This function prints the status of the goal 
 * @note **reacheable** or **not reacheable** or **under proccesing**
 * 
 * 
 **/
void read_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){

//if_condition is used to avoid the problem when the status.list is empty (not allocated memory)
 
if (!msg->status_list.empty()){
status= msg->status_list[0].status;
}
if(input ==1){
    if(status == 1){
      printf(" Goal is under proccesing   status = %d \n",status);
    	}
    if(status == 3){
      printf(" Goal Reached!!  status = %d \n",status);
    	}
    if(status == 4){
      printf(" Goal is not Reacheable   status = %d \n",status);
    	}
  }
}

//subscriber callback to topic cmd_vel
/**
 * @brief Callback function of the subscriber sub_vel 
 * @see sub_vel  
 * @param msg stores the angular and linear velocity of the robot
 * @return nothing 
 * 
 **/
void read_velocity(const geometry_msgs::Twist::ConstPtr& msg){
linear_x= msg->linear.x;
angular_z= msg->angular.z;
}

//subscriber callback of topic(user_topic) containing the data entred by the user

/**
 * @brief Callback function of the subscriber sub_user 
 * @see sub_user
 * @param msg the message entred by the user 
 * 
 * @return nothing
 * 
 * @note Prints the mode choosen by the user. 
 * @attention msg: is a customized message has the following format: 
 * @attention int32 user
 * @attention float32 x
 * @attention float32 y 
 * @warning It is called **only** when the user enters new data.
 **/
void user_input(const assignement_3::User::ConstPtr& msg)
{
 //printf("status = %d \n",status);
 input= msg->user;
 x= msg->x;
 y= msg->y;  

 if(input==1){
   printf("You are in Mode (1),Reaching a goal  \n");
   printf("you Can cancel by pressing '4' \n");
   target.goal.target_pose.pose.position.x = x;
   target.goal.target_pose.pose.position.y = y;
   target.goal.target_pose.header.frame_id = "map";
   target.header.frame_id = "map";
   target.goal.target_pose.pose.orientation.w = 1.0;
   pub_goal.publish(target); 
 }
 
 if(input==2){
   printf("You are in Mode (2), NOT ASSISTED Driving  \n");
   pub_cancel.publish(cancel);
 }
 if(input==3){
   printf("You are in Mode (3), ASSISTED Driving \n");
 }
 if(input==4){
   printf("canceling the target \n");
 // publish to this topic an empty msg to cancel the goal 
   pub_cancel.publish(cancel);
 }
}


/**
 * @brief Callback function of the subscriber sub_scan 
 * @see sub_scan  
 * @param msg of type LaserScan 
 * 
 * @return nothing
 * 
 * @note Contains the code of all the 4 **modes** entred by the user 
 * @warning It is called contineously because the robot publishes 
 * @warning data to the /LaserScan topic very rapidely 
 * @warning this is why the main part of the code is putted her.
 **/
//subscriber to scan topic
void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
// the if statement used to cancel the target if user enters Mode2
if(input==2){ 
  pub_cancel.publish(cancel);
 	}

// these variables are used to declare the initial index of each range  
int i1=300;
int i2=420;
int i3=0;
//the min is used to returne the most closest obstacl in eache range 
float min1,min2,min3,min4,min5,min6;
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




/**
 * @brief the main function 
 * @param argc
 * @param argv
 * @return 0
 * 
 * Description:
 * initialize all the publishers and subscribers
 **/
int main (int argc, char **argv)
{
ros::init(argc, argv, "pubsub");
ros::NodeHandle nh;
sub_scan = nh.subscribe("/scan", 10, robotCallback);
sub_vel = nh.subscribe("/cmd_vel", 10, read_velocity);
sub_goal_status = nh.subscribe("/move_base/status", 100, read_status);
sub_user = nh.subscribe("user_topic", 10, user_input);
pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal> ("/move_base/goal", 100); 
pub_cancel = nh.advertise<actionlib_msgs::GoalID> ("/move_base/cancel", 100); 
pub_vel = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10); 
ros::spin();

return 0;
}
