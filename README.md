# r2auto_nav
ROS2 auto_nav code for EG2310, sem2 AY21/22

Welcome to group4's repo for EG2310, and this is all our code for our Beta Gasoline robot.

Please follow our step by step guide to start up your system. We are assuming that you are using the exact same hardware as us, such as our launcher and loading system, and our electrical connections.

## Preparation
#### Do the following things to setup your laptop and turtlebot

 1. Follow the instructions here to setup your laptop and turtlebot https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
 2. Follow this for instructions on setting up i2c communication on the turtlebot's raspberry pi https://ask.wingware.com/question/3/i2c-problem-with-remote-raspberry-pi/
 3. Follow this for instructions on setting up gpiozero and pigpio pin factory (so that the servo used will not spasm) https://gpiozero.readthedocs.io/en/stable/remote_gpio.html




## File allocation and building of workspaces
clone our repository into your Home directory.

    git clone https://github.com/Magmanat/r2auto_nav.git
 Then shift the entire folder hardware_bringup into your turtlebot3_ws using scp, then build the workspace, for example
 

    scp -r path_to_r2auto_nav/turtlebot3_ws/src/hardware_bringup ubuntu@(ip-address-of-pi):~/turtlebot3/src
    ssh ubuntu@(ip-address-of-pi)
    cd turtlebot3_ws
    colcon build
  
  cd into colcon_ws and colcon build the workspace to setup the ros package on your laptop
  

    cd path_to_r2auto_nav/colcon_ws
    colcon build
  Your should be done setting up after this, now its time to have fun with our package
  
  ## System check
Firstly, we will use the factory_test package to ensure that everything is running properly, we will ssh into the turtlebot and startup all the hardware publishers and subscribers.

#### For your Turtlebot
##### In one terminal
    ssh ubuntu@(ip-address-of-pi)
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
##### In another terminal
    ssh ubuntu@(ip-address-of-pi)
    ros2 launch hardware_bringup hardware.launch.py
  
Finally, we will run the script in your laptop to check that the whole system is working
#### For your Laptop
##### In one terminal
    ros2 run auto_nav factory_test

Follow the instructions printed on your terminal, and if everything works out fine, it means the system is ready to go.

## Running auto_nav in a maze
Now we will get to try the robot in an actual maze, and see it complete its mission, prepare two terminals for your Turtlebot and prepare 3 terminals for your Laptop.
Similarly, we will start all the hardware in the Turtlebot first.
#### For your Turtlebot
##### In one terminal
    ssh ubuntu@(ip-address-of-pi)
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
##### In another terminal
    ssh ubuntu@(ip-address-of-pi)
    ros2 launch hardware_bringup hardware.launch.py
  
Finally, we will run the commands to start the auto navigation
#### For your Laptop
##### First terminal
    ros2 launch turtlebot3_cartographer cartographer.launch.py
Ensure that you see rviz open up, laserscan and map data will be able to be seen on the rviz window.
##### Second terminal
    ros2 run auto_nav map2base
Ensure that the transforms are properly being published by observing the terminal for any error messages that print.
##### Third terminal
    ros2 run auto_nav wall_follow
Finally, after running this command, the robot should move around and perform its mission autonomously, moving around the maze and stopping at an nfc loading bay, which it will then stop and wait to be loaded and button to be pressed to continue with its mission. After moving one entire round around the maze, and also being loaded, it will then search for a thermal target, which when found, the turtlebot would start centering towards the target and finally when the target is approximately 15 cm away from the turtlebot, it would stop and start the firing sequence.




