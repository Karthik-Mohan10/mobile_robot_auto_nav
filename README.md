# robot_auto_nav
Robot_auto_nav is a ROS package for implementing autonomous navigation in a Differential Drive Mobile Robot. The Mobile robot system is driven by two Nissei BLDC Motors, controlled through Arduino microcontroller. The whole algorithm runs in [Robot Operating System (ROS)](https://www.ros.org/)and [ROS Navigation Stack](http://wiki.ros.org/navigation/Tutorials/RobotSetup) has been setup to implement autonomous navigation.

**File Descriptions**

- AGV_Motor_Control.ino is the Arduino code for motor control. The RPM control of motor is done through PWM inputs. Any motor can be used and controlled using this code by appropriately modifying the motor pin configurations

- src/ros_arduino.py file helps for ROS - Arduino communication with less latency.
The manual control of Mobile robot using direction keystrokes are possible with the help of src/agv_keyboard_control_ros.py file

- Use the following files for doing Mapping, Localization, & Navigation;
  - For Mapping: launch/AIO_gmapping.launch
  - For Localization: launch/amcl_demo.launch
  - For Navigation: launch/move_base_demo.launch




