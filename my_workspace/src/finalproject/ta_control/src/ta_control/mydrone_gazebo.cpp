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

PID mypid_x             (0.03125, 1.0, -1.0, 1.0, 0.600, 0.00000); // GANTI!!! sesuai rate class!!!
PID mypid_y             (0.03125, 1.0, -1.0, 0.750, 0.600, 0.00000);
PID mypid_z             (0.03125, 1.0, -1.0, 0.350, 0.015, 0.00000);


MyDrone::MyDrone(double pixel_w, double pixel_h, double hfov, double vfov, double Kp, double Ki, double Kd, std::string object) : rate(32){
    arming_client       = nh.serviceClient<mavros_msgs::CommandBool>    ("mavros/cmd/arming");
    set_mode_client     = nh.serviceClient<mavros_msgs::SetMode>        ("mavros/set_mode");
    local_pos_pub       = nh.advertise<geometry_msgs::PoseStamped>      ("mavros/setpoint_position/local", 10);
    vel_pub             = nh.advertise<geometry_msgs::Twist>            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    atom_cmd_sub        = nh.advertise<std_msgs::Bool>                  ("atom_cmd/", 10);

    state_sub           = nh.subscribe<mavros_msgs::State>              ("mavros/state", 10, &MyDrone::state_cb, this);
    current_pos_sub     = nh.subscribe<geometry_msgs::PoseStamped>      ("mavros/local_position/pose", 10, &MyDrone::localpos_cb, this);

    vision_outter_sub   = nh.subscribe<geometry_msgs::PointStamped>     ("camera/data/outter", 10, &MyDrone::outter_pixel_cb, this);
    vision_middle_sub   = nh.subscribe<geometry_msgs::PointStamped>     ("camera/data/middle", 10, &MyDrone::middle_pixel_cb, this);
    vision_inner_sub    = nh.subscribe<geometry_msgs::PointStamped>     ("camera/data/inner", 10, &MyDrone::inner_pixel_cb, this);
    
    outter_bool_sub     = nh.subscribe<std_msgs::String>                ("camera/bool/outter", 10, &MyDrone::outter_bool_cb, this);
    middle_bool_sub     = nh.subscribe<std_msgs::String>                ("camera/bool/middle", 10, &MyDrone::middle_bool_cb, this);
    inner_bool_sub      = nh.subscribe<std_msgs::String>                ("camera/bool/inner", 10, &MyDrone::inner_bool_cb, this);


    tfmini_sub          = nh.subscribe<sensor_msgs::Range>              ("tfmini", 10, &MyDrone::tfmini_cb, this);
    odom_sub            = nh.subscribe<nav_msgs::Odometry>              ("mavros/local_position/odom", 10, &MyDrone::odom_cb, this);

    est_dist_pub        = nh.advertise<geometry_msgs::PoseStamped>      ("est_dist", 10);

    laser_scan_pub      = nh.subscribe<sensor_msgs::LaserScan>          ("scan", 10, &MyDrone::laser_scan_cb, this);

    compass_sub         = nh.subscribe<std_msgs::Float64>               ("mavros/global_position/compass_hdg", 10, &MyDrone::compass_cb, this);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV Connected");
    _pixel_w     = pixel_w;
    _pixel_h     = pixel_h;
    _hfov        = hfov;
    _vfov        = vfov;
    nilaiP      = Kp;
    nilaiI      = Ki;
    nilaiD      = Kd;
    which_object = object;
    atom_cmd.data = false;

};

MyDrone::~MyDrone(){
    ROS_INFO("ALL DONE!!!");
}


void MyDrone::arm(){
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 7;
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
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed) && !is_takeoff){
        PID pidyaw (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
        if (current_position.pose.position.z <= 0.2){
            vel.linear.x    = mypid_x.calculate(start_x, current_position.pose.position.x);
            vel.linear.y    = mypid_y.calculate(start_y, current_position.pose.position.y);
            vel.linear.z    = mypid_z_takeoff.calculate(3, tfmini_val.range); //nanti coba ubah pake lidar
        }
        else if(current_position.pose.position.z >= 0.301){
            vel.linear.x    = mypid_x.calculate(start_x + 3, current_position.pose.position.x);
            vel.linear.y    = mypid_y.calculate(start_y, current_position.pose.position.y);
            vel.linear.z    = mypid_z_takeoff.calculate(5, tfmini_val.range); //nanti coba ubah pake lidar    
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

        if (current_position.pose.position.z <= 2.7 ){  //ini juga ubah pake lidar
            ROS_INFO("Altitude = %f Yaw = %f", current_position.pose.position.z, current_position.pose.orientation.z);
            cek_waktu = ros::Time::now();
      	}
        else{
            ROS_INFO("Sudah takeoff!");
            is_takeoff = true;
        }
        ROS_INFO("laser = %f", laser_scan_val.ranges[0]);
        vel.angular.x   = 0;
        vel.angular.y	= 0;
	    vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
}

void MyDrone::start_mission(){
    ROS_INFO("Masuk misiiii");
    cek_waktu = ros::Time::now();
    atom_cmd.data = true;
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if (is_middle_detected.data == "true"){
                double x = estimate_x(middle_pixel_val.point.y, current_position.pose.position.z);
                double y = estimate_y(middle_pixel_val.point.x, current_position.pose.position.z);
                
                if((x <= -0.5) || (x >= 0.5) || ((y <= -0.2) || (y >= 0.2))){
                    centering_outter("local", 2.7);
                    ROS_INFO("x = %f    y = %f", x, y);
                }
                else{
                    ROS_INFO("Masuk misi......");
                    // centering_outter("local", 3.0);
                    break;
                    // arm_cmd.request.value = false;
                }
            
        }
        else if((ros::Time::now() - cek_waktu <= ros::Duration(2.0)) && (is_middle_detected.data == "false")){ //fungsi untuk trigger hunting supaya berhenti
            hunting(3.0);
        }
        else{
            keep_in_place(3.0);
        }
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("start yaw = %f", start_yaw);
    }  
}

void MyDrone::mission(){
    cek_waktu = ros::Time::now();
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){

        if(current_position.pose.position.z <= 0.5){
            ROS_INFO("LANDINGGGGGGGGGGGGGGG");
            auto_land();
            arm_cmd.request.value = false;
            break;
        }
        if ((current_position.pose.position.z >= 2.51) && (current_position.pose.position.z <= 3.3)){
            ROS_INFO("misi altitude 2.5 - 3.3");
            if(is_middle_detected.data == "true"){
                centering_outter("local", 2.2);
                cek_waktu = ros::Time::now();
            }
            else{
               if((ros::Time::now() - cek_waktu) <= ros::Duration(2.0)){ // FIX ** hunting disini ganggu, pas kamera ilang vision malah jd kehunting
                    // hunting(2.5);
                }
                else{keep_in_place(3.0);}
            } 
        }
        
        else if ((current_position.pose.position.z >= 1.71) && (current_position.pose.position.z <= 2.5)){
            ROS_INFO("misi altitude 1.5 - 2.5");
            if (is_middle_detected.data == "true"){
                centering_middle("local", 1.2);
                cek_waktu = ros::Time::now();
            }
            else {
                if((ros::Time::now() - cek_waktu) <= ros::Duration(2.0)){
                    // hunting(2.0);
                }
                else{keep_in_place(3.0);}
            }
        }

        else if ((current_position.pose.position.z >= 1.01) && (current_position.pose.position.z <= 1.7)){
            ROS_INFO("misi altitude 0.5 - 1.5");
            if (is_inner_detected.data == "true"){
                centering_inner("local", 0.5);
                cek_waktu = ros::Time::now();
            }
            else {
                if((ros::Time::now() - cek_waktu) <= ros::Duration(2.0)){
                    // hunting(1.5);
                }
                else{keep_in_place(3.0);}
            }
        }
        else if ((current_position.pose.position.z >= 0.1001) && (current_position.pose.position.z <= 1.0)){
            ROS_INFO("misi altitude <= 0.5");
            if (is_inner_detected.data == "true"){
                centering_inner2("local", 0.00);
                cek_waktu = ros::Time::now();
            }
            else {
                if((ros::Time::now() - cek_waktu) <= ros::Duration(2.0)){
                    // hunting(1.5);
                }
                else{keep_in_place(3.0);}
            }
        }
            
        else if ((is_outter_detected.data == "false") && (is_middle_detected.data == "false") && (is_inner_detected.data == "false")){
            if(ros::Time::now() - cek_waktu < ros::Duration(3.0)){
                ROS_INFO("mana ne ga ketemu");
                // hunting(3.0);
            }
            else{
               keep_in_place(3.0);
            }
        }


        //vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
    
}
// ----------------------- FUNCTION -----------------------

void MyDrone::centering_outter(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (middle_pixel_val.point.y, alt);
    double y = estimate_y           (middle_pixel_val.point.x, alt);
    est_dist.pose.position.x = x + current_position.pose.position.x;
    est_dist.pose.position.y = y + current_position.pose.position.y;
    est_dist_pub.publish(est_dist);
    // PID mypid_x_vision_outter(0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    // PID mypid_y_vision(0.03125, 1.0, -1.0, nilaiP, nilaiD, nilaiI);
    PID mypid_x_vision_outter       (0.03125, 1.0, -1.0, 1.25, 0.0, 0.7);
    PID mypid_y_vision_outter       (0.03125, 1.0, -1.0, 0.7, 0.0, 0.5);
    PID pidyaw                      (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_outter.calculate(x + current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_outter.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, current_position.pose.position.z);
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
}

void MyDrone::centering_middle(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (middle_pixel_val.point.y, alt);
    double y = estimate_y           (middle_pixel_val.point.x, alt);
    est_dist.pose.position.x = x + current_position.pose.position.x;
    est_dist.pose.position.y = y + current_position.pose.position.y;
    est_dist_pub.publish(est_dist);
    PID mypid_x_vision_middle       (0.03125, 1.0, -1.0, 1.5, 0.0, 1.0);
    PID mypid_y_vision_middle       (0.03125, 1.0, -1.0, 0.7, 0.0, 0.5);
    PID pidyaw                      (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_middle.calculate(x + current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_middle.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, current_position.pose.position.z);
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
}

void MyDrone::centering_inner(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (inner_pixel_val.point.y, alt);
    double y = estimate_y           (inner_pixel_val.point.x, alt);
    est_dist.pose.position.x = x + current_position.pose.position.x;
    est_dist.pose.position.y = y + current_position.pose.position.y;
    est_dist_pub.publish(est_dist);
    PID mypid_x_vision_inner        (0.03125, 1.0, -1.0, 2.25, 0.0, 1.0);
    PID mypid_y_vision_inner        (0.03125, 1.0, -1.0, 1.0, 0.0, 0.6);
    PID pidyaw                      (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_inner.calculate(0.2 + x + current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_inner.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, current_position.pose.position.z);
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
}

void MyDrone::centering_inner2(std::string sensorType, double desired_alt){
    double alt;
    if (sensorType == "tfmini"){
        alt = tfmini_val.range;
    }
    else if (sensorType == "local"){
        alt = current_position.pose.position.z;
    }
    double x = estimate_x           (inner_pixel_val.point.y, alt);
    double y = estimate_y           (inner_pixel_val.point.x, alt);
    est_dist.pose.position.x = x + current_position.pose.position.x;
    est_dist.pose.position.y = y + current_position.pose.position.y;
    est_dist_pub.publish(est_dist);
    PID mypid_x_vision_inner2        (0.03125, 1.0, -1.0, 2.5, 0.0, 1.25);
    PID mypid_y_vision_inner2        (0.03125, 1.0, -1.0, 1.0, 0.0, 0.6);
    PID pidyaw                       (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    vel.linear.x    = mypid_x_vision_inner2.calculate(0.2 + x + current_position.pose.position.x, 
                                                    current_position.pose.position.x);
    vel.linear.y    = -1 * mypid_y_vision_inner2.calculate(y + current_position.pose.position.y, current_position.pose.position.y);    
    vel.linear.z    = mypid_z.calculate(desired_alt, current_position.pose.position.z);
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
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
void MyDrone::hunting(double alt){
    PID pidyaw                      (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    ROS_INFO("huntinggg");
    vel.linear.x        = mypid_x.calculate(current_position.pose.position.x + 1.0, current_position.pose.position.x);
    vel.linear.y        = mypid_y.calculate(0, current_position.pose.position.y);
    vel.linear.z        = mypid_z.calculate(alt, current_position.pose.position.z);
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
    PID pidyaw                      (0.03125, 1.0, -1.0, 0.5, 0.1, 0.25);
    ROS_INFO("diem dahh...");
    vel.linear.x        = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
    vel.linear.y        = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
    vel.linear.z        = mypid_z.calculate(alt, current_position.pose.position.z);
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
    double value = tan(scale * 3.14159/180) * (height - 0.3);
    return value;
}

double MyDrone::estimate_y(double data_pixel, double height){
    double scale = (data_pixel / _pixel_h) * _vfov;
    double value = tan(scale * 3.14159/180) * (height - 0.3);
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
void MyDrone::laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    laser_scan_val      = *msg;
}
void MyDrone::compass_cb(const std_msgs::Float64::ConstPtr& msg){
    compass_val         = *msg;
}

//---------------------- CALLBACK FUNCTION ----------------------

