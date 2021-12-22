# Decription of the program
* A launche file has been used to run diffrent nodes and other lauche files.
* the nodes : * robot_controller
              * User_Interface
              * teleop_twist_keyboard
* launche files : * move_base.launch 
                  * simulation_gmapping.launch*
                 


# How to run it 
1- run the 2D simulator by   "rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world"
2- run the server by  "rosrun my_srv server "
3- run the user interface by "rosrun ass2 ui_node"
4- run the controller node by " rosrun ass2 controller "
# How to use the user interface 
the user need to enter integer numbers to control the robot.
enter 1 to reset
enter 2 to slowdown
enter 3 to speedup
# PSEUDO_CODE OF THE CONTROLLER.CPP 
headers
declare the publisher and services 
declare user_input() 
    // it is a function used to return the value of the integer entred by the user
declare robotCalback()
    initialize integer variables to starting index for each range (i used 6 ranges ) i1...i6
    initialize a float to stor the minnimum value returned by eache range (min1 .. min6)
    if(the user enter 1) {call the reset_position service }
    if(the user enter 1) {call the customited service }
    if(the user enter 3) {call the customited service }
    publish a specific angular and linear velocity for each range 
int main()
    intialize publisher and clients
