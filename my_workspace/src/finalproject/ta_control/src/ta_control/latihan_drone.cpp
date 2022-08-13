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
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "ta_control/mydrone.hpp"
#include "ta_control/pid.hpp"


PID mypid_z_takeoff     (0.03125, 1.0, -1.0, 1.000, 0.355, 0.01500);

PID mypid_yaw           (0.03125, 3.0, -3.0, 0.30, 0.05, 0.00000);

PID mypid_x             (0.03125, 1.0, -1.0, 0.750, 0.600, 0.00000); // GANTI!!! sesuai rate class!!!
PID mypid_y             (0.03125, 1.0, -1.0, 0.750, 0.600, 0.00000);
PID mypid_z             (0.03125, 1.0, -1.0, 0.350, 0.015, 0.00000);

MyDrone::MyDrone(double pixel_w, double pixel_h, double hfov, double vfov, double Kp, double Ki, double Kd, std::string object) : rate(32){
    arming_client       = nh.serviceClient<mavros_msgs::CommandBool>    ("mavros/cmd/arming");
    set_mode_client     = nh.serviceClient<mavros_msgs::SetMode>        ("mavros/set_mode");
    local_pos_pub       = nh.advertise<geometry_msgs::PoseStamped>      ("mavros/setpoint_position/local", 10);
    vel_pub             = nh.advertise<geometry_msgs::Twist>            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    state_sub           = nh.subscribe<mavros_msgs::State>              ("mavros/state", 10, &MyDrone::state_cb, this);
    current_pos_sub     = nh.subscribe<geometry_msgs::PoseStamped>      ("mavros/local_position/pose", 10, &MyDrone::localpos_cb, this);

    vision_outter_sub   = nh.subscribe<geometry_msgs::PointStamped>     ("camera/data/outter", 1, &MyDrone::outter_pixel_cb, this);
    vision_middle_sub   = nh.subscribe<geometry_msgs::PointStamped>     ("camera/data/middle", 1, &MyDrone::middle_pixel_cb, this);
    vision_inner_sub    = nh.subscribe<geometry_msgs::PointStamped>     ("camera/data/inner", 1, &MyDrone::inner_pixel_cb, this);
    
    outter_bool_sub     = nh.subscribe<std_msgs::String>                ("camera/bool/outter", 1, &MyDrone::outter_bool_cb, this);
    middle_bool_sub     = nh.subscribe<std_msgs::String>                ("camera/bool/middle", 1, &MyDrone::middle_bool_cb, this);
    inner_bool_sub      = nh.subscribe<std_msgs::String>                ("camera/bool/inner", 1, &MyDrone::inner_bool_cb, this);


    tfmini_sub          = nh.subscribe<sensor_msgs::Range>              ("tfmini", 1, &MyDrone::tfmini_cb, this);
    odom_sub            = nh.subscribe<nav_msgs::Odometry>              ("mavros/local_position/odom", 10, &MyDrone::odom_cb, this);

    compass_sub         = nh.subscribe<std_msgs::Float64>               ("mavros/global_position/compass_hdg", 10, &MyDrone::compass_cb, this);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV Connected");
    _pixel_w    = pixel_w;
    _pixel_h    = pixel_h;
    _hfov       = hfov;
    _vfov       = vfov;
    nilaiP      = Kp;
    nilaiI      = Ki;
    nilaiD      = Kd;
    which_object = object;

};

MyDrone::~MyDrone(){
    ROS_INFO("ALL DONE!!!");
}

// ----------------------- FUNCTION -----------------------
void MyDrone::arm(){
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode   = "OFFBOARD";
    arm_cmd.request.value               = true;
    ros::Time last_request 		        = ros::Time::now();
    while(ros::ok() && !(current_state.mode == "OFFBOARD" && current_state.armed)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    is_takeoff = false;
}

void MyDrone::take_off(){
    start_yaw = 0.0;
    double start_x = current_position.pose.position.x;
    double start_y = current_position.pose.position.y;
    ROS_INFO("Taking off!");
    tfmini_seq_past = 0;
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed) && !is_takeoff){
        tfmini_seq_now = tfmini_val.header.seq;
        PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
        if(((tfmini_seq_now > tfmini_seq_past) && (tfmini_val.range < 5.0)) || (tfmini_seq_now == 0 && tfmini_seq_past ==0)){

            tfmini_seq_past = tfmini_val.header.seq;

            if (tfmini_val.range <= 0.3){
                vel.linear.x    = mypid_x.calculate(start_x, current_position.pose.position.x);
                vel.linear.y    = mypid_y.calculate(start_y, current_position.pose.position.y);
                vel.linear.z    = mypid_z_takeoff.calculate(3, tfmini_val.range); //nanti coba ubah pake lidar
            }
            else if(tfmini_val.range >= 0.301){
                vel.linear.x    = mypid_x.calculate(start_x, current_position.pose.position.x);
                vel.linear.y    = mypid_y.calculate(start_y, current_position.pose.position.y);
                vel.linear.z    = mypid_z_takeoff.calculate(3, tfmini_val.range); //nanti coba ubah pake lidar    
            }


            if (compass_tf(compass_val.data) >= (0.0 + 0.001)){
                vel.angular.z   = -1 * pidyaw.calculate(0.0001, compass_tf(compass_val.data));
            } 
            else if (compass_tf(compass_val.data) <= (0.0 - 0.001)){
                vel.angular.z   = -1 * pidyaw.calculate(-0.0001, compass_tf(compass_val.data));
            }
            
            else{
                vel.angular.z   = 0.0;
            } 


            if (tfmini_val.range <= 2.7 ){  //ini juga ubah pake lidar
                ROS_INFO("Altitude = %f Yaw = %f", tfmini_val.range, current_position.pose.orientation.z);
                cek_waktu = ros::Time::now();
            }
            else{
                ROS_INFO("Sudah takeoff!");
                is_takeoff = true;
            }
        }
        else {
            ROS_INFO("NO TFMINI DATA");
            auto_land();
            break;
        }
        vel.angular.x   = 0;
        vel.angular.y	= 0;
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
}

void MyDrone::start_mission(){
    ROS_INFO("Masuk misiiii");
    ROS_INFO("testt");
    cek_waktu = ros::Time::now();
    tfmini_seq_past = 0;
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        tfmini_seq_now = tfmini_val.header.seq;
        if(((tfmini_seq_now > tfmini_seq_past) && (tfmini_val.range < 5.0)) || (tfmini_seq_now == 0 && tfmini_seq_past ==0)){
            
            tfmini_seq_past = tfmini_val.header.seq;

            if(which_object == "outter"){

                if (is_middle_detected.data == "true"){
                    centering_outter("tfmini", 2.2);
                }
                else{ 
                    std::cout << "keep di 2.2" << std::endl;
                    keep_in_place(2.2);
                }
            }

            else if(which_object == "middle"){
                if (is_middle_detected.data == "true"){
                    centering_middle("tfmini", 1.2);   
                }
                else{
                    std::cout << "keep di 1.2" << std::endl;
                    keep_in_place(1.2);
                }
            }

            else if(which_object == "inner"){
                if (is_inner_detected.data == "true"){
                        centering_inner("tfmini", 0.5);
                }
                else{
                    std::cout << "keep di 0.5" << std::endl;
                    keep_in_place(0.5);
                }
            }
        }
        else {
            ROS_INFO("NO TFMINI DATA");
            auto_land();
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }  
}

// ------------------------------ FUNCTION -----------------------------------------

void MyDrone::centering_outter(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (middle_pixel_val.point.y * 1.53846154, alt);
    double y = estimate_y           (middle_pixel_val.point.x * 1.15384615, alt);
    // PID mypid_x_vision_outter       (0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    // PID mypid_y_vision_outter       (0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    PID mypid_x_vision_outter       (0.03125, 1.0, -1.0, 1.25, 0.0, 0.7);
    PID mypid_y_vision_outter       (0.03125, 1.0, -1.0, 0.7, 0.0, 0.5);
    PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_outter.calculate(x + current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_outter.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, alt);
    if (compass_tf(compass_val.data) >= (0.0 + 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(0.0001, compass_tf(compass_val.data));
    } 
    else if (compass_tf(compass_val.data) <= (0.0 - 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(-0.0001, compass_tf(compass_val.data));
    }
    else {
            vel.angular.z   = 0.0;
        }      
    vel_pub.publish(vel);
    ROS_INFO("Centering OUTTER");
    ROS_INFO("x = %f    y = %f    z = %f", x, y, alt);
}

void MyDrone::centering_middle(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (middle_pixel_val.point.y * 1.53846154, alt);
    double y = estimate_y           (middle_pixel_val.point.x * 1.15384615, alt);
    // PID mypid_x_vision_middle       (0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    // PID mypid_y_vision_middle       (0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    PID mypid_x_vision_middle       (0.03125, 1.0, -1.0, 1.5, 0.0, 1.0);
    PID mypid_y_vision_middle       (0.03125, 1.0, -1.0, 0.7, 0.0, 0.5);
    PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_middle.calculate(x + current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_middle.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, alt);
    if (compass_tf(compass_val.data) >= (0.0 + 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(0.0001, compass_tf(compass_val.data));
    } 
    else if (compass_tf(compass_val.data) <= (0.0 - 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(-0.0001, compass_tf(compass_val.data));
    }
    else {
            vel.angular.z   = 0.0;
        }     
    vel_pub.publish(vel);
    ROS_INFO("Centering MIDDLE");
    ROS_INFO("x = %f    y = %f    z = %f", x, y, alt);
}

void MyDrone::centering_inner(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (inner_pixel_val.point.y * 1.53846154, alt);
    double y = estimate_y           (inner_pixel_val.point.x * 1.15384615, alt);
    // PID mypid_x_vision_inner       (0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    // PID mypid_y_vision_inner       (0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    PID mypid_x_vision_inner       (0.03125, 1.0, -1.0, 2.25, 0.0, 1.0);
    PID mypid_y_vision_inner       (0.03125, 1.0, -1.0, 1.0, 0.0, 0.6);
    PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_inner.calculate(0.1 + x + current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_inner.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, alt);
    if (compass_tf(compass_val.data) >= (0.0 + 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(0.0001, compass_tf(compass_val.data));
    } 
    else if (compass_tf(compass_val.data) <= (0.0 - 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(-0.0001, compass_tf(compass_val.data));
    }
    else {
            vel.angular.z   = 0.0;
        }     
    vel_pub.publish(vel);
    ROS_INFO("Centering INNER");
    ROS_INFO("x = %f    y = %f    z = %f", x, y, alt);
}


void MyDrone::centering_inner2(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (inner_pixel_val.point.y * 1.53846154, alt);
    double y = estimate_y           (inner_pixel_val.point.x * 1.15384615, alt);
    PID mypid_x_vision_inner2        (0.03125, 1.0, -1.0, 2.5, 0.0, 1.25);
    PID mypid_y_vision_inner2        (0.03125, 1.0, -1.0, 1.0, 0.0, 0.6);
    PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_inner2.calculate(0.1 + x + current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_inner2.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, alt);
    if (compass_tf(compass_val.data) >= (0.0 + 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(0.0001, compass_tf(compass_val.data));
    } 
    else if (compass_tf(compass_val.data) <= (0.0 - 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(-0.0001, compass_tf(compass_val.data));
    }
    else {
            vel.angular.z   = 0.0;
        }     
    vel_pub.publish(vel);
    ROS_INFO("SIAP-SIAP LANDING");
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
void MyDrone::hunting(double alt){
    PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    ROS_INFO("huntinggg");
    vel.linear.x        = mypid_x.calculate(current_position.pose.position.x + 1.0, current_position.pose.position.x);
    vel.linear.y        = mypid_y.calculate(0, current_position.pose.position.y);
    vel.linear.z        = mypid_z.calculate(alt, tfmini_val.range);
    if (compass_tf(compass_val.data) >= (0.0 + 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(0.0001, compass_tf(compass_val.data));
    } 
    else if (compass_tf(compass_val.data) <= (0.0 - 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(-0.0001, compass_tf(compass_val.data));
    }
    else {
            vel.angular.z   = 0.0;
        }     
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

void MyDrone::keep_in_place(double alt){
    PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x        = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y        = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
    vel.linear.z        = mypid_z.calculate(alt, tfmini_val.range);
    if (compass_tf(compass_val.data) >= (0.0 + 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(0.0001, compass_tf(compass_val.data));
    } 
    else if (compass_tf(compass_val.data) <= (0.0 - 0.001)){
        vel.angular.z   = -1 * pidyaw.calculate(-0.0001, compass_tf(compass_val.data));
    }
    else {
            vel.angular.z   = 0.0;
        }     
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

bool MyDrone::is_offboard(){
    if(current_state.mode == "OFFBOARD"){
        return true;
    }
    else{
        return false;
    }
}

void MyDrone::auto_land(){
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(offb_set_mode);
}

double MyDrone::estimate_x(double data_pixel, double height){
    double scale = (data_pixel / _pixel_w) * _hfov;
    double value = tan(scale * 3.14159/180) * height;
    return value;
}

double MyDrone::estimate_y(double data_pixel, double height){
    double scale = (data_pixel / _pixel_h) * _vfov;
    double value = tan(scale * 3.14159/180) * height;
    return value;
}

double MyDrone::compass_tf(double val){
    if ((val >= 90.0) && (val <= 360.0)){
        val = val - 90;
    }
    else if ((val >= 0.0) && (val <= 89.99999)){
        val = val + 270;
    }
    if ((val <= 360.0) && (val >= 180.0)){
        val = (val - 360) / 180;
    }
    else if ((val <= 179.99999) && (val >= 0.0)){
        val = val / 180;
    }
    return val;
}

//---------------------- CALLBACK FUNCTION ----------------------
void MyDrone::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void MyDrone::localpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}


void MyDrone::outter_pixel_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    outter_pixel_val    = *msg;
}
void MyDrone::middle_pixel_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    middle_pixel_val    = *msg;
}
void MyDrone::inner_pixel_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    inner_pixel_val    = *msg;
}

void MyDrone::outter_bool_cb(const std_msgs::String::ConstPtr& msg){
    is_outter_detected  = *msg;
}
void MyDrone::middle_bool_cb(const std_msgs::String::ConstPtr& msg){
    is_middle_detected  = *msg;
}
void MyDrone::inner_bool_cb(const std_msgs::String::ConstPtr& msg){
    is_inner_detected  = *msg;
}

void MyDrone::tfmini_cb(const sensor_msgs::Range::ConstPtr& msg){
    tfmini_val          = *msg;
}
void MyDrone::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odometry_val        = *msg;
}
void MyDrone::compass_cb(const std_msgs::Float64::ConstPtr& msg){
    compass_val         = *msg;
}
//---------------------- CALLBACK FUNCTION ----------------------