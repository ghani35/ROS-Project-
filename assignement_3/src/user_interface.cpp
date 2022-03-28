//this node is used to publish the entred value by the use through a costumized message

/**
 * @file user_interface.cpp
 * @brief User interface node to allow the user to **choose a mode** and **set a goal**
 * @author Bakour Abdelghani bakourabdelghani1999@gmail.com 
 * @version 1.0
 * @date 25/12/2021
 * 
 * Description:
 * This node asks the user to enter an **integer** value.
 * each integer value points to a different **mode** 
 * 
 * 		 
 *
 * 
 * 
 * Publishers to:<BR>
 *   /user_topic
 * 
 * @note These are the mode's reprentations of each integer value:
 * @note 	1 : Set a goalpiont
 * @note 	2 : Drive the robot by the keyboard **Not assestive**
 * @note 	3 : Drive the robot by hte keyboard **Assistive**
 * @note 	4 : Cancel the goal
 *
 * @warning in case the user chose mode **1**, he must *set a goal point*
 *
 * @attention The goal point is in **2D**
 * @attention has the form (x,y), while x and y are floats
 * @see controller.cpp 
 *
 **/

#include "ros/ros.h"
#include "assignement_3/User.h"
#include <sstream>


ros::Publisher pub;///<Global publisher to the topic */user_topic*, to publish the mode.

/**
 * @brief main function 
 * @param argc 
 * @param argv
 * @return 0
 * @note It prints the **mode** that has been chosen 
 * @note and the coordinates of the goal point if it is set 
 * 
 **/

main (int argc, char **argv)
{
float x,y; 
int mode;  

ros::init(argc, argv, "User_node");
ros::NodeHandle nh;
pub = nh.advertise<assignement_3::User> ("user_topic", 1); 
ros::Rate loop_rate(10);

while (ros::ok()){
    assignement_3::User input;
    printf("Please choose MODE by entring integer value : \n");
    printf("'1' for Mode1 , '2' for Mode2 , '3' for Mode3 : \n"); 
    printf("Cancel your Goal by pressing '4' : \n");
    scanf("%d",&mode);
if(mode!=4){
    printf("You choose Mode (%d) \n", mode);
     }
if(mode==1){
    printf("you are in mode 1 please enter a target  \n");
    scanf("%f", &x);
    scanf("%f", &y);
    printf("your target is (%f , %f)\n",x , y);
     }
if(mode==4){
    printf("Target Canceled");
    }
            
input.user = mode;
input.x=x;
input.y=y;
pub.publish(input);
ros::spinOnce();
loop_rate.sleep();
}
return 0;
}

