# Research Track final assignment 
This is a ROS architecture for the control of a mobile robot in the Gazebo environment. The software relies on the move_base and gmapping packages for localizing the robot and plan the motion. The architecture gets the user request, and lets the robot execute one of the pre-defined behaviors accordingly, along with Simulataneous Localization and Mapping (SLAM), path planning, and collision avoidance.

The user can choose among 4 possible modalities:
* mode 1: the user can set a target for the robot to reach
* mode 2: the user can drive the robot by the keyboard (relying on the package teleop_twist_keyboard)
* mode 3: the same as mode 2, but in this case the robot gives you some help, and does not let you crash the walls 
* mode 4: the user can cancel the goal at any time 

The user interface is impelemnted in both **cpp code**, located in the source file, and in **juputer notebook**.
As well as the whole architectue of the assignment is documented by **doxygen**

## Instructions for running the project
### Starting the simulation
you can start the simulation by following these steps
* 1- Make sur to be in the source file of your workspace /work_space/src
* 2- clone the repository https://github.com/ghani35/assignement_3.git
* 3- build the workspace /catkin_make
* 4- launch the launch file **solution.launch** by roslaunch solution.launch 
     by launching the file, the follwing things starts automatically 
     * rviz
     * Gazebo
     * user_interface.cpp
     * robot_controller.cpp 
     * teleop_twist_keyboard.py 
 
 ### Starting jupyter notebook 
 you can start the jupyter notebook by 
* 1- make sure you are in /work_space/assignment3
* 2- excute the command juputer notebook --allow-root
  

## Documentation: Doxygen
The architecture of the assignment is documented by using Doxygen, you can access the documentations by this link 
https://ghani35.github.io/assignement_3/ 

## Jupyter Notebook 
The Jupyter Notebook is an open source web application that you can use to create and share documents that contain live code,
equations, visualizations, and text.

The user interface of the final assignemnemnt is implemented in a different way using Jupyter notebook, to have a better 
graphical user interface for cotrolling the robot, as well as a better visualization of the data such as: 
* line graph of the position of the robot
* line graph of the distance of the obstacles 
* a bar graph of the reached and unreached goals


## Decription of files
### Simulator: Gazebo
Gazebo is the 3D simulator for ROS

### Visualizer: rviz
rviz is a 3D tool for ROS Visualization. It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information. By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor inputs to planned (or unplanned) actions

### Controller node: robot_controller 
It is a .cpp code that impelments four possible modalities of the robot, it interacts direly with the user interface node by subscribing to /user_topic,
You can read the pseudo code for a deep understanding.

#### Psudo code of the robot_controller.cpp
* headers 
* declare variables 
  * x           < float to stor the x postiion of the target >
  * y           < float to stor the y position of the target > 
  * linear_x    < float to stor the velocity of the robot along x axis >
  * angular_z   < float to stor the velocity of the robot along z axis >
  * input       < integer to stor the mode entred by the user >
  * status      < integer to stor the status of the goal >
  
* declare publishers 
  * pub_cancel
  * pub_vel
  * pub_goal
 
* subscriber callback function to topic /cmd_vel < read_velocity >

* subscriber callback function to topic /move_base/status < read_status > 
  * if (status_list(0).status is not empty)
    * read the status filed of the message and stor it variable int status
  * if ( input = 1 )
    * if ( status = 1 )
      * print goal is under proccessing
    * if ( status = 3 )
      * Goal Reached !!
    * if ( status = 4 )
      * Goal is not reacheable 

* subscriber callback function to topic /user_topic < user_input >
  * if ( mode 1 ) 
    * print a message 
    * publish the goal < pub_goal >
  * if ( mode 2 )
    * print a message
    * cancel the target < pub_cancel >
  * if ( mode 3 )
    * print a message
  * if ( mode 4 )
    * print a message
    * cancel the target < pub_cancel >
    
* subscriber callback function to topic /scan < robotcallback > 
  * decalre i1,i2,i3
  * declare min1,min2,min3  to store the minimum distance in front,left,right
  * for loops to calculate min1,min2,min3
  * if ( mode 3 ) 
    * if ( close obstacle in front and robot go forward )
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
    * sub_goal_status

* run by launching a launche file < solution.launche > to run diffrent nodes and other lauche files.
  * the nodes : 
    * robot_controller
    * User_Interface
    * teleop_twist_keyboard
  * launche files : 
    * move_base.launch 
    * simulation_gmapping.launch            


### User interface node: user_interface.cpp
It is a .cpp code that allows the user to choose one modalitie by entring an integer value (1,2,3 or 4) such that: 
1 for mode1, 2 for mode2, 3 for mode 3, and 4 for mode4.
the data entred by the user is published in /user_topic 

#### Pseudo code of User_Interface

* declare variables 
  * x           < float to stor the x postiion of the target >
  * y           < float to stor the y position of the target >
  * mode        < integer to store the value of the mode entred > 
* initialize and declare publisher < pub >
* print the entred mode 
* while (roscore is running) 
  * declare variable of type < User.msg >  
  * print some messages on the screen
  * if ( mode 1 ) 
    * ask the user to enter a target 
  * if ( mode 4 )
    * print canceling
  * fill the message
  * publish the message


