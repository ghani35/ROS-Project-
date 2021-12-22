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
