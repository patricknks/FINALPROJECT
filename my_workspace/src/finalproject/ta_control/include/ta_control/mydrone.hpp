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
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

class MyDrone{
    private:
        ros::NodeHandle                 nh;
        ros::Rate                       rate;
        bool                            is_takeoff;
        ros::Time                       cek_waktu;     
        ros::Time                       waktu1;
        ros::Time                       waktu2;


        // Service client, subscriber, publisher
        ros::ServiceClient              arming_client;
        ros::ServiceClient              set_mode_client;
        ros::Publisher                  local_pos_pub;
        ros::Publisher                  vel_pub;

        ros::Subscriber                 state_sub;
        ros::Subscriber                 current_pos_sub;
        
        ros::Subscriber                 vision_outter_sub;
        ros::Subscriber                 vision_middle_sub;
        ros::Subscriber                 vision_inner_sub;

        ros::Subscriber                 outter_bool_sub;
        ros::Subscriber                 middle_bool_sub;
        ros::Subscriber                 inner_bool_sub;

        ros::Subscriber                 compass_sub;

        ros::Subscriber                 tfmini_sub;
        ros::Subscriber                 odom_sub;
        ros::Publisher                  atom_cmd_sub;
        ros::Publisher                  est_dist_pub;
        ros::Subscriber                 laser_scan_pub;
        

        // Publish variable
        mavros_msgs::CommandBool        arm_cmd;
        mavros_msgs::SetMode            offb_set_mode;
        geometry_msgs::PoseStamped      pose;
        geometry_msgs::Twist            vel;
        std_msgs::Bool                  atom_cmd;
        geometry_msgs::PoseStamped      est_dist;

        // Subscribe variable
        mavros_msgs::State              current_state;
        std_msgs::Int16MultiArray       object_position;
        geometry_msgs::PoseStamped      current_position;
        geometry_msgs::PoseStamped      last_position;
        std_msgs::String                is_outter_detected;
        std_msgs::String                is_middle_detected;
        std_msgs::String                is_inner_detected;

        geometry_msgs::PointStamped     outter_pixel_val;
        geometry_msgs::PointStamped     middle_pixel_val;
        geometry_msgs::PointStamped     inner_pixel_val;
        sensor_msgs::Range              tfmini_val;
        nav_msgs::Odometry              odometry_val;
        sensor_msgs::LaserScan          laser_scan_val;

        std_msgs::Float64               compass_val;


        // Callback subscribe
        void    state_cb(const mavros_msgs::State::ConstPtr& msg);
        void    localpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
       
        void    outter_pixel_cb(const geometry_msgs::PointStamped::ConstPtr& msg);
        void    middle_pixel_cb(const geometry_msgs::PointStamped::ConstPtr& msg);
        void    inner_pixel_cb(const geometry_msgs::PointStamped::ConstPtr& msg);
        
        void    outter_bool_cb(const std_msgs::String::ConstPtr& msg);
        void    middle_bool_cb(const std_msgs::String::ConstPtr& msg);
        void    inner_bool_cb(const std_msgs::String::ConstPtr& msg);

        void    tfmini_cb(const sensor_msgs::Range::ConstPtr& msg);
        void    odom_cb(const nav_msgs::Odometry::ConstPtr& msg);

        void    laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg);

        void    compass_cb(const std_msgs::Float64::ConstPtr& msg);

        //Mission variable 
        double                          _pixel_w;
        double                          _pixel_h;
        double                          _hfov;
        double                          _vfov; 
        double                          nilaiP;
        double                          nilaiI;
        double                          nilaiD;
        std::string                     which_object;
        double                          start_yaw;
        uint32_t                        tfmini_seq_past;
        uint32_t                        tfmini_seq_now;

    public:
        MyDrone(double pixel_w, double pixel_h, double hfov, double vfov, double Kp, double Ki, double Kd, std::string object);                      //constructor
        ~MyDrone();                     //destructor

        void arm();
        void take_off();
        void start_mission();
        void mission();
        void hunting(double alt);
        void keep_in_place(double alt);
        void centering_outter(std::string sensorType, double desired_alt);
        void centering_middle(std::string sensorType, double desired_alt);
        void centering_middle2(std::string sensorType, double desired_alt);
        void centering_inner(std::string sensorType, double desired_alt);
        void centering_inner2(std::string sensorType, double desired_alt);
        void auto_land();
        double estimate_x(double data_pixel, double height);
        double estimate_y(double data_pixel, double height);
        bool is_offboard();
        double compass_tf(double val);
};