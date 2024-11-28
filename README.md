# rtic_project
Differential drive robot control

In this repository you will find an implementation of two main controllers 
which are the sliding mode controller and a proportional controller.

## To use the sliding mode controller:

Execute the sliding_mode_controller.py script
    
     ` python sliding_mode_controller.py `

## To use the proportional controller:

Execute the proportional_controller.py script
        
      ` python proportional_controller.py `

# ROS IMPLEMENTATION

For the ros implementation you can use the trajectory package which is a ros implementation 
of this controllers.

## Note : This package must be used inside a catkin_workspace 
## You may also need to clone the following repository 
   [ https://github.com/ROBOTIS-GIT/turtlebot3.git ]  
   to get informations about the turtlebot3

To run either of the controllers use the command 

\ `rosrun trajectory sliding_mode.py `
\ `rosrun trajectory proportional_controller.py `
\ `rosrun trajectory target.py  `
