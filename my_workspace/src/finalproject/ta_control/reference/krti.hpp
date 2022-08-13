#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <boost/array.hpp>
#include <geometry_msgs/PointStamped.h>


class Soeromiber{
private:
    ros::NodeHandle     nh;
    ros::Rate           rate;
    bool                is_takeoff;
    ros::Time           cek_waktu;
    std::string         dropper_status;

    // Service client, subscriber, publisher
    ros::ServiceClient      arming_client;
    ros::ServiceClient      set_mode_client;
    ros::Publisher          local_pos_pub;
    ros::Publisher          vel_pub;
    ros::Publisher          servo1_pub;
    ros::Publisher          servo2_pub;
    ros::Publisher          servo3_pub;
    ros::Publisher          dropper_pub;
    ros::Subscriber         state_sub;
    ros::Subscriber         current_pos_sub;
    ros::Subscriber         vision_sub;
    ros::Subscriber         tfmini_bottom_sub;
    ros::Subscriber         tfmini_front_sub;

    // Publish variable
    mavros_msgs::SetMode            offb_set_mode;
    mavros_msgs::CommandBool        arm_cmd;
    geometry_msgs::PoseStamped      pose;
    geometry_msgs::Twist            vel;
    std_msgs::UInt8                 servo1_ang;
    std_msgs::UInt8                 servo2_ang;
    std_msgs::UInt8                 servo3_ang;
    std_msgs::UInt8                 dropper_pos;

    // Subscribe variable
    mavros_msgs::State              current_state;
    std_msgs::Int16MultiArray       object_position;
    geometry_msgs::PoseStamped      current_position;
    geometry_msgs::PoseStamped      last_position;
    sensor_msgs::Range              tfmini_alt;
    sensor_msgs::Range              tfmini_front;
    geometry_msgs::PointStamped     vision_pixel_val;

    // Callback subscribe
    void    state_cb(const mavros_msgs::State::ConstPtr& msg);
    void    vision_cb(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void    localpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void    tfmini_bottom_cb(const sensor_msgs::Range::ConstPtr& msg);
    void    tfmini_front_cb(const sensor_msgs::Range::ConstPtr& msg);

    //Waypoint Variable
    std::vector<double>     GEDUNG_A;
    std::vector<double>     GEDUNG_B;
    std::vector<double>     GEDUNG_C;
    std::vector<double>     ELP;
    std::vector<double>     START_POINT;
    std::vector<double>     END_POINT;

    //Mission variable
    std::map<int, std::vector<double>>      WP_MISI;
    int                                     MISI;
    bool                                    CENTERING;
    bool                                    CEK_LAST_POSITION;
    float                                   default_yaw;

public:
    Soeromiber(int x, int y);
    ~Soeromiber();

    void arm();
    void take_off();
    void start_point();
    void end_point();
    void start_mission_with_qr();
    void start_mission_without_qr();
    void landing();
    void drop_payload(std::vector<double> a);
    void drop_payload_B(std::vector<double> a);

    void goto_wp(std::vector<double> x);
    void goto_wp_b();
    void goto_ELP(std::vector<double> x);
    void centering_A(double z, std::string s);
    void centering_B(double z, std::string s);
    void centering_C(double z, std::string s);
    void centering_ELP(double z, std::string s);
    void auto_land();
    void set_servo(int a, int b=0);
    void set_dropper(std::string s);
    double dist_to_wp(std::vector<double> x);
    double dist_to_elp(std::vector<double> x);
    bool is_offboard();
    
};
