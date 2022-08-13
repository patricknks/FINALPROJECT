#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "krti.hpp" 
#include "pid.hpp"
#include "krti.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "krti_2021");

    int x, y;
    std::cout << "Masukkan Angka! Kombinasi Misi dan Lokasi ELP ";
    std::cin >> x >> y;

    Soeromiber galih(x,y);

    char hhh;
    std::cout << "Kalau sudah siap, masukkin sembarang char lalu enter!!! ";
    std::cin >> hhh;

    galih.arm();

    galih.take_off();

    galih.start_point();

    galih.start_mission_without_qr();

    galih.end_point();

    galih.landing();

    galih.set_servo(0, 0);

    ROS_INFO("SELESAI");
    return 0;
}

