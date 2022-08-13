#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "ta_control/mydrone.hpp"
#include "ta_control/pid.hpp"
#include "ta_control/mydrone_gazebo.cpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "main_gazebo");

    double x,y,z;
    std::string object;
    
   

    std::cout << "~ Whenever You're Ready ~" << std::endl;
    std::cout << "nilai KP = ";
    std::cin >> x;
    std::cout << "\n";
    
    std::cout << "nilai Ki = ";
    std::cin >> y;
    std::cout << "\n";

    std::cout << "nilai Kd = ";
    std::cin >> z;
    std::cout << "\n";

    std::cout << "which object = ";
    std::cin >> object;
    std::cout << "\n";

    MyDrone quadcopter(640.0, 480.0, 40.9, 31.3, x, y, z, object);
    quadcopter.arm();

    quadcopter.take_off();

    quadcopter.start_mission();

    quadcopter.mission();



    return 0;
}