//this node is used to publish the entred value by the use through a costumized message

#include "ros/ros.h"
#include "assignement_3/User.h"
#include <sstream>

int main (int argc, char **argv)
{
float x,y;
int mode;

ros::init(argc, argv, "User_node");
ros::NodeHandle nh;
ros::Publisher pub = nh.advertise<assignement_3::User> ("user_topic", 1); 
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
    printf("your target is (%f , %f)\n",&x , &y);
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

