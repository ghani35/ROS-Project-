# Decription of the program

* run by launching a launche file to run diffrent nodes and other lauche files.
  * the nodes : 
    * robot_controller
    * User_Interface
    * teleop_twist_keyboard
  * launche files : 
    * move_base.launch 
    * simulation_gmapping.launch            

## Pseudo code of User_Interface

* declare variables x,y,mode
* initialize and declare publisher pub
* print the entred mode 
* while (roscore is running) 
  * declare variable of type User.msg 
  * print some messages on the screen
  * if ( mode 1 ) 
    * ask the user to enter a target 
  * if ( mode 4 )
    * print canceling
  * fill the message
  * publish the message

## Pseudo code of robot_controller 
* headers 
* declare variables 
  * x,y,linear_x,angular_z,input
* declare publishers 
  * pub_cancel
  * pub_vel
  * pub_goal
* subscriber callback function read_velocity
* subscriber callback function user_input
  * if ( mode 1 ) 
    * print a message 
    * publish the goal pub_goal
  * if ( mode 2 )
    * print a message
    * cancel the target pub_cancel
  * if ( mode 3 )
    * print a message
  * if ( mode 4 )
    * print a message
    * cancel the target pub_cancel
* subscriber callback function robotcallback
  * decalre i1,i2,i3
  * declare min1,min2,min3  to store the minimum distance in front,left,right
  * for loops to calculate min1,min2,min3
  * if ( mode 3 ) 
    * if ( close obstacle in front robot go forward )
      * publish 0 linear velocity
    * if ( close obstacle in left and robot turn left )
      * publish 0 angular velocity
    * if ( close obstacle in right and robot turn right )
      * publish 0 linear velocity
* main function 
  * initialize the publishers 
    * pub_cancel
    * pub_goal
    * pub_vel
  * initialize subscribers 
    * sub_scan
    * sub_vel
    * sub_user
