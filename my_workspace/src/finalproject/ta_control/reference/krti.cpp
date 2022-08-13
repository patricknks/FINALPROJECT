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
#include "krti.hpp"
#include "pid.hpp"
#include <math.h>


PID mypid_z_takeoff(0.03125, 1.0, -1.0, 1.000, 0.355, 0.01500);

PID mypid_yaw(0.03125, 3.0, -3.0, 0.30, 0.05, 0.00000);

PID mypid_x(0.03125, 1.0, -1.0, 0.750, 0.600, 0.00000); // GANTI!!! sesuai rate class!!!
PID mypid_y(0.03125, 1.0, -1.0, 0.750, 0.600, 0.00000);
PID mypid_z(0.03125, 1.0, -1.0, 0.350, 0.015, 0.00000);

PID mypid_x_vision(0.03125, 1, -1, 0.00090, 0.00090, 0); // GANTI!!! sesuai rate class!!!
PID mypid_y_vision(0.03125, 1, -1, 0.00090, 0.00090, 0);
PID mypid_z_vision(0.03125, 0.5, -0.5, 0.35000, 0.01500, 0);

PID mypid_x_vision_elp(0.03125, 1, -1, 0.0010, 0.0011, 0);
PID mypid_y_vision_elp(0.03125, 1, -1, 0.0010, 0.0011, 0);

PID mypid_x_landing(0.03125, 1.0, -1.0, 0.500, 0.150, 0.00000);
PID mypid_y_landing(0.03125, 1.0, -1.0, 0.500, 0.150, 0.00000);

Soeromiber::Soeromiber(int x, int y) : rate(32){
    arming_client       = nh.serviceClient<mavros_msgs::CommandBool>    ("mavros/cmd/arming");
    set_mode_client     = nh.serviceClient<mavros_msgs::SetMode>        ("mavros/set_mode");
    local_pos_pub       = nh.advertise<geometry_msgs::PoseStamped>      ("mavros/setpoint_position/local", 10);
    vel_pub             = nh.advertise<geometry_msgs::Twist>            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    servo1_pub          = nh.advertise<std_msgs::UInt8>                 ("servo1", 10);
    servo2_pub          = nh.advertise<std_msgs::UInt8>                 ("servo2", 10);
    servo3_pub          = nh.advertise<std_msgs::UInt8>                 ("servo3", 10);
    dropper_pub         = nh.advertise<std_msgs::UInt8>                 ("dropper", 10);
    state_sub           = nh.subscribe<mavros_msgs::State>              ("mavros/state", 10, &Soeromiber::state_cb, this);
    current_pos_sub     = nh.subscribe<geometry_msgs::PoseStamped>      ("mavros/local_position/pose", 10, &Soeromiber::localpos_cb, this);
    vision_sub          = nh.subscribe<std_msgs::Int16MultiArray>       ("vision", 10, &Soeromiber::vision_cb, this);
    tfmini_bottom_sub   = nh.subscribe<sensor_msgs::Range>              ("tfmini/bottom", 10, &Soeromiber::tfmini_bottom_cb, this);
    tfmini_front_sub    = nh.subscribe<sensor_msgs::Range>              ("mavros/distance_sensor/hrlv_ez4_pub", 10, &Soeromiber::tfmini_front_cb, this);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV Connected");
    

    dropper_status  = "CLOSED";
    // default_yaw     = -0.75;

    nh.param("/krti/default_yaw", default_yaw, float());
    nh.param("/krti/elp/" + std::to_string(y), ELP, std::vector<double>());
    nh.param("/krti/gedung/bawah/a", GEDUNG_A, std::vector<double>());
    nh.param("/krti/gedung/bawah/b", GEDUNG_B, std::vector<double>());
    nh.param("/krti/gedung/bawah/c", GEDUNG_C, std::vector<double>());

    switch(x){
        case 1:
            WP_MISI = {{1, GEDUNG_A}, {2, GEDUNG_B}, {3, GEDUNG_C}};
            nh.param("/krti/gedung/atas/a", START_POINT, std::vector<double>());
            nh.param("/krti/gedung/atas/c", END_POINT, std::vector<double>());
	        break;
        case 2:
            WP_MISI = {{1, GEDUNG_A}, {3, GEDUNG_B}, {2, GEDUNG_C}};
            nh.param("/krti/gedung/atas/a", START_POINT, std::vector<double>());
            nh.param("/krti/gedung/atas/b", END_POINT, std::vector<double>());
            break;
        case 3:
            WP_MISI = {{2, GEDUNG_A}, {1, GEDUNG_B}, {3, GEDUNG_C}};
            nh.param("/krti/gedung/atas/b", START_POINT, std::vector<double>());
            nh.param("/krti/gedung/atas/c", END_POINT, std::vector<double>());
	        break;
        case 4:
            WP_MISI = {{2, GEDUNG_A}, {3, GEDUNG_B}, {1, GEDUNG_C}};
            nh.param("/krti/gedung/atas/c", START_POINT, std::vector<double>());
            nh.param("/krti/gedung/atas/b", END_POINT, std::vector<double>());
	        break;
        case 5:
            WP_MISI = {{3, GEDUNG_A}, {1, GEDUNG_B}, {2, GEDUNG_C}};
            nh.param("/krti/gedung/atas/b", START_POINT, std::vector<double>());
            nh.param("/krti/gedung/atas/a", END_POINT, std::vector<double>());
	        break;
        case 6:
            WP_MISI = {{3, GEDUNG_A}, {2, GEDUNG_B}, {1, GEDUNG_C}};
            nh.param("/krti/gedung/atas/c", START_POINT, std::vector<double>());
            nh.param("/krti/gedung/atas/a", END_POINT, std::vector<double>());
            break;
    }
    ROS_INFO("Gedung A = %f %f %f", GEDUNG_A[0], GEDUNG_A[1], GEDUNG_A[2]);
    ROS_INFO("Gedung B = %f %f %f", GEDUNG_B[0], GEDUNG_B[1], GEDUNG_B[2]);
    ROS_INFO("Gedung C = %f %f %f", GEDUNG_C[0], GEDUNG_C[1], GEDUNG_C[2]);
    ROS_INFO("Start Point = %f %f %f", START_POINT[0], START_POINT[1], START_POINT[2]);
    ROS_INFO("End Point   = %f %f %f", END_POINT[0], END_POINT[1], END_POINT[2]);
    ROS_INFO("Default yaw = %f", default_yaw);
}

Soeromiber::~Soeromiber(){
}

void Soeromiber::arm(){
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

void Soeromiber::take_off(){
    ROS_INFO("Taking off!");
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed) && !is_takeoff){
        vel.linear.x    = mypid_x.calculate(0, current_position.pose.position.x);
        vel.linear.y    = mypid_y.calculate(0, current_position.pose.position.y);
        vel.linear.z    = mypid_z_takeoff.calculate(2.75, current_position.pose.position.z);
        vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
        vel.angular.x   = 0;
        vel.angular.y	= 0;
	    vel_pub.publish(vel);

        if (current_position.pose.position.z < 2.250){
            ROS_INFO("Altitude = %f Yaw = %f", current_position.pose.position.z, current_position.pose.orientation.z);
            cek_waktu = ros::Time::now();
      	}
        else if (ros::Time::now() - cek_waktu > ros::Duration(1.0)){
            ROS_INFO("Sudah takeoff!");
            is_takeoff = true;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    vel.angular.z = 0;
}

void Soeromiber::start_point(){
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        goto_wp(START_POINT);
        if (dist_to_wp(START_POINT) > 0.3){
            cek_waktu = ros::Time::now();
            std::string ss = "Starting Point. Jarak = " + std::to_string(dist_to_wp(START_POINT));
            ROS_INFO(ss.c_str());
        }
        else if (ros::Time::now() - cek_waktu > ros::Duration(0.5)){
            ROS_INFO("Sudah di titik start!");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void Soeromiber::end_point(){
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        goto_wp(END_POINT);
        if (dist_to_wp(END_POINT) > 0.3){
            cek_waktu = ros::Time::now();
            std::string ss = "Ending Point. Jarak = " + std::to_string(dist_to_wp(END_POINT));
            ROS_INFO(ss.c_str());
        }
        else if (ros::Time::now() - cek_waktu > ros::Duration(0.5)){
            ROS_INFO("Sudah di titik end!");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void Soeromiber::start_mission_with_qr(){ // ENGGAK DIPAKAI
    std::vector<double> GEDUNG_A, GEDUNG_B, GEDUNG_C, GEDUNG_A_DROP, GEDUNG_B_DROP, GEDUNG_C_DROP;
    GEDUNG_A = {-4.0,  7.0,  2.0};
    GEDUNG_B = { 0.0,  7.0,  1.0};
    GEDUNG_C = { 4.0,  7.0,  1.5};
    GEDUNG_A_DROP = {-4.0,  8.0,  2.5};
    GEDUNG_B_DROP = { 0.0,  8.0,  1.5};
    GEDUNG_C_DROP = { 4.0,  8.0,  2.0};
    std::map<int, std::vector<double>> WP_MISI, WP_DROP;
    // switch (x){
    //     case 1:
    //         WP_MISI = {{1,      GEDUNG_A}, {2,      GEDUNG_B}, {3,      GEDUNG_C}};
    //         WP_DROP = {{1, GEDUNG_A_DROP}, {2, GEDUNG_B_DROP}, {3, GEDUNG_C_DROP}};
    //     case 2:
    //         WP_MISI = {{1,      GEDUNG_A}, {3,      GEDUNG_B}, {2,      GEDUNG_C}};
    //         WP_DROP = {{1, GEDUNG_A_DROP}, {3, GEDUNG_B_DROP}, {2, GEDUNG_C_DROP}};
    //     case 3:
    //         WP_MISI = {{2,      GEDUNG_A}, {1,      GEDUNG_B}, {3,      GEDUNG_C}};
    //         WP_DROP = {{2, GEDUNG_A_DROP}, {1, GEDUNG_B_DROP}, {3, GEDUNG_C_DROP}};
    //     case 4:
    //         WP_MISI = {{2,      GEDUNG_A}, {3,      GEDUNG_B}, {1,      GEDUNG_C}};
    //         WP_DROP = {{2, GEDUNG_A_DROP}, {3, GEDUNG_B_DROP}, {1, GEDUNG_C_DROP}};
    //     case 5:
    //         WP_MISI = {{3,      GEDUNG_A}, {1,      GEDUNG_B}, {2,      GEDUNG_C}};
    //         WP_DROP = {{3, GEDUNG_A_DROP}, {1, GEDUNG_B_DROP}, {2, GEDUNG_C_DROP}};
    //     case 6:
    //         WP_MISI = {{3,      GEDUNG_A}, {2,      GEDUNG_B}, {1,      GEDUNG_C}};
    //         WP_DROP = {{3, GEDUNG_A_DROP}, {2, GEDUNG_B_DROP}, {1, GEDUNG_C_DROP}};
    // }
    int MISI = 1;
    bool CHECKING = true;
    bool DROPPING = false;
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed) && MISI < 4){
        if (CHECKING){
            goto_wp(WP_MISI[MISI]);
            if (dist_to_wp(WP_MISI[MISI]) > 0.3){
                cek_waktu = ros::Time::now();
                std::string ss = "Jarak = " + std::to_string(dist_to_wp(WP_MISI[MISI]));
                ROS_INFO(ss.c_str());
            }
            else if (ros::Time::now() - cek_waktu > ros::Duration(1.0)){
                MISI += 1;
                ROS_INFO("Sudah di titik!");
                if (MISI == 3){
                    CHECKING = false;
                    MISI = 1;
                }
                
            }    
        }
        else{
            if(!DROPPING){
                goto_wp(WP_DROP[MISI]);
                if (dist_to_wp(WP_DROP[MISI]) > 0.3){
                    cek_waktu = ros::Time::now();
                    std::string ss = "Jarak = " + std::to_string(dist_to_wp(WP_DROP[MISI]));
                    ROS_INFO(ss.c_str());
                }
                else if (ros::Time::now() - cek_waktu > ros::Duration(0.5)){
                    DROPPING = true;
                }    
            }
            else {
                ////////////   LANJUTIN DI SINI    ////////////
                ////////////   CENTERING VISION    ////////////
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    
}

void Soeromiber::start_mission_without_qr(){
    MISI = 1;
    CENTERING = false;
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed) && MISI < 4){
        if (!CENTERING){
            goto_wp(WP_MISI[MISI]);
            if (dist_to_wp(WP_MISI[MISI]) > 0.5){
                cek_waktu = ros::Time::now();
                std::string ss = "Misi " + std::to_string(MISI) + ". Jarak = " + std::to_string(dist_to_wp(WP_MISI[MISI]));
                ROS_INFO(ss.c_str());
            }
            else if (WP_MISI[MISI] == GEDUNG_B){ 
                if (ros::Time::now() - cek_waktu > ros::Duration(2.0)){
                    if (object_position.data[2] != -1){
                        ROS_INFO("Sudah di titik misi.\nMulai Centering...");
                        cek_waktu = ros::Time::now();
                        CENTERING = true;
                    }
                    else{
                        ROS_INFO("Belum dapet data");
                    }
                }
                
                else{
                    // vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);  
                    ROS_INFO("Delay baru sampai gedung...");
                }
            }
            else{
                if (ros::Time::now() - cek_waktu > ros::Duration(2.0)){
                    if (object_position.data[2] != -1){
                        ROS_INFO("Sudah di titik misi.\nMulai Centering...");
                        cek_waktu = ros::Time::now();
                        CENTERING = true;
                    }
                    else{
                        ROS_INFO("Belum dapet data");
                    }
                }
                else{
                    // vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);  
                    ROS_INFO("Delay baru sampai gedung...");
                }
            }
        }
        else if (WP_MISI[MISI] == GEDUNG_B){
            drop_payload_B(WP_MISI[MISI]);
        }
        else {
            drop_payload(WP_MISI[MISI]);
        }   
        ros::spinOnce();
        rate.sleep();
    }
}

void Soeromiber::landing(){ 
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if(dist_to_elp(ELP) > 0.3){
            goto_ELP(ELP);
            ROS_INFO("Belum centering. Jarak = %f", dist_to_elp(ELP));
            //cek_waktu = ros::Time::now();
        }
        else{
            ROS_INFO("Mulai centering");
            break;
        }

        // if(object_position.data[5] != -1){
        //     //goto_wp({current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z});
        //     //if (ros::Time::now() - cek_waktu > ros::Duration(1.0)){
        //     ROS_INFO("Mulai centering");
        //     break;
        //     //}
        // }
        ros::spinOnce();
        rate.sleep();
    }
    
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if(current_position.pose.position.z < 0.5){
            auto_land();
            ROS_INFO("AUTO LAND");
            break;
        }
        else{
            if(object_position.data[5] != -1){
                if(abs(object_position.data[3]) < 75 && abs(object_position.data[4]) < 75){
                    centering_ELP(0, "tfmini");
                    ROS_INFO("Centering ke bawah...");
                }
                else{
                    centering_ELP(current_position.pose.position.z, "realsense");
                    ROS_INFO("Centring di atas...");
		        }
            }
            else{
                goto_wp({current_position.pose.position.x, tfmini_front.range, current_position.pose.position.z});
		        ROS_INFO("Gak detek landing pad...");
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void Soeromiber::goto_wp(std::vector<double> x){
    vel.linear.x = mypid_x.calculate(x[0], current_position.pose.position.x);
    vel.linear.y = mypid_y.calculate(tfmini_front.range, x[1]);
    vel.linear.z = mypid_z.calculate(x[2], current_position.pose.position.z);
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

void Soeromiber::goto_wp_b(){
    vel.linear.x  = mypid_x.calculate(0 , current_position.pose.position.x);
    vel.linear.y  = mypid_y.calculate(tfmini_front.range, 0.75);
    // vel.linear.z = mypid_z.calculate(x[2], current_position.pose.position.z);
    vel.linear.z  = mypid_z.calculate(1.75, current_position.pose.position.z);
    // vel.angular.z = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

void Soeromiber::goto_ELP(std::vector<double> x){
    vel.linear.x = mypid_x_landing.calculate(x[0], current_position.pose.position.x);
    vel.linear.y = mypid_y_landing.calculate(x[1], current_position.pose.position.y);
    vel.linear.z = mypid_z.calculate(x[2], current_position.pose.position.z);
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

double Soeromiber::dist_to_wp(std::vector<double> x){
    double jarak_x = abs(current_position.pose.position.x - x[0]);
    double jarak_y = abs(tfmini_front.range - x[1]);
    double jarak_z = abs(current_position.pose.position.z - x[2]);
    double jarak = sqrt(pow(jarak_x, 2) + pow(jarak_y, 2) + pow(jarak_z, 2));
    return jarak;
}

double Soeromiber::dist_to_elp(std::vector<double> x){
    double jarak_x = abs(current_position.pose.position.x - x[0]);
    double jarak_y = abs(current_position.pose.position.y - x[1]);
    double jarak_z = abs(current_position.pose.position.z - x[2]);
    double jarak = sqrt(pow(jarak_x, 2) + pow(jarak_y, 2) + pow(jarak_z, 2));
    return jarak;
}

void Soeromiber::centering_A(double z, std::string s){
    if (object_position.data[2] != -1){
        vel.linear.x = mypid_x_vision.calculate(object_position.data[0], 0);
        vel.linear.y = mypid_y_vision.calculate(-10, object_position.data[1]);
    }
    else{
        vel.linear.x = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
    }

    if (s == "tfmini"){
        vel.linear.z = mypid_z_vision.calculate(z, tfmini_alt.range);
    }
    else if (s == "realsense"){
        vel.linear.z = mypid_z_vision.calculate(z, current_position.pose.position.z);
    }

    // vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);        
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

void Soeromiber::centering_B(double z, std::string s){
    if (object_position.data[2] != -1){
        vel.linear.x = mypid_x_vision.calculate(object_position.data[0], 0);
        vel.linear.y = mypid_y_vision.calculate(-25, object_position.data[1]);
    }
    else{
        vel.linear.x = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
    }

    if (s == "tfmini"){
        vel.linear.z = mypid_z_vision.calculate(z, tfmini_alt.range);
    }
    else if (s == "realsense"){
        vel.linear.z = mypid_z_vision.calculate(z, current_position.pose.position.z);
    }

    // vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

void Soeromiber::centering_C(double z, std::string s){
    if (object_position.data[2] != -1){
        vel.linear.x = mypid_x_vision.calculate(object_position.data[0], 0);
        vel.linear.y = mypid_y_vision.calculate(40, object_position.data[1]);
    }
    else{
        vel.linear.x = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
    }

    if (s == "tfmini"){
        vel.linear.z = mypid_z_vision.calculate(z, tfmini_alt.range);
    }
    else if (s == "realsense"){
        vel.linear.z = mypid_z_vision.calculate(z, current_position.pose.position.z);
    }

    // vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);        
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

void Soeromiber::centering_ELP(double z, std::string s){
    if (object_position.data[5] != -1){
        vel.linear.x = mypid_x_vision_elp.calculate(object_position.data[3], 0);
        vel.linear.y = mypid_y_vision_elp.calculate(0, object_position.data[4]);
    }
    else{
        vel.linear.x = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
    }

    if (s == "tfmini"){
        vel.linear.z = mypid_z_vision.calculate(z, tfmini_alt.range);
    }
    else if (s == "realsense"){
        vel.linear.z = mypid_z_vision.calculate(z, current_position.pose.position.z);
    }

    // vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);        
    vel_pub.publish(vel);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

void Soeromiber::auto_land(){
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(offb_set_mode);
}

void Soeromiber::drop_payload(std::vector<double> a){
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if((a == GEDUNG_A && abs(object_position.data[0]) < 57 && object_position.data[1] < 47 && object_position.data[1] > -67 && object_position.data[2] != -1) || (a == GEDUNG_C && abs(object_position.data[0]) < 37 && object_position.data[1] < 77 && object_position.data[1] > 17 && object_position.data[2] != -1)){
            if (a == GEDUNG_A){                 
                set_servo(1, 1);
                ROS_INFO("%f %f", object_position.data[0], object_position.data[1]);
                ROS_INFO("Drop Gedung A");
            }
            if (a == GEDUNG_C){                  
                set_servo(1, 3);
                ROS_INFO("%f %f", object_position.data[0], object_position.data[1]);
                ROS_INFO("Drop Gedung C");
            }
            break;
        }
        else{ 
            if(object_position.data[2] != -1){
                if (a == GEDUNG_A){
                    centering_A(WP_MISI[MISI][2], "realsense");
                    ROS_INFO("Centering di atas...");
                }
                else if (a == GEDUNG_C){
                    centering_C(WP_MISI[MISI][2], "realsense");
                    ROS_INFO("Centering di atas...");
                }
            }
            else{
                vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
                goto_wp({current_position.pose.position.x, tfmini_front.range, current_position.pose.position.z});
                ROS_INFO("Gak detek pad...");
	        }
        }
        ros::spinOnce();
        rate.sleep();
    }
    cek_waktu = ros::Time::now();
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if (ros::Time::now() - cek_waktu < ros::Duration(1.0)){
            vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
            goto_wp({current_position.pose.position.x, tfmini_front.range, current_position.pose.position.z});
            ROS_INFO("Delay habis drop");
        }
        else if (dist_to_wp(WP_MISI[MISI]) > 0.3 && MISI < 3){
	        ROS_INFO("Balik ke titik gedung...");
            goto_wp(WP_MISI[MISI]);
        }
	    else{
	        break;
	    }
        ros::spinOnce(); 
	    rate.sleep();
    }
    set_servo(0, 1);
    set_servo(0, 3);
    cek_waktu = ros::Time::now();
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if ((ros::Time::now() - cek_waktu < ros::Duration(0.5)) && MISI < 3){
            ROS_INFO("Delay di titik gedung");
            vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
            goto_wp({current_position.pose.position.x, tfmini_front.range, current_position.pose.position.z});
        }
        else {
            MISI = MISI + 1;
            CENTERING = false;
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void Soeromiber::drop_payload_B(std::vector<double> a){
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if(object_position.data[1] < 25 && object_position.data[1] > -75 && abs(object_position.data[0]) < 50){ 
	    set_dropper("reset");
            set_servo(1, 2);
            ROS_INFO("Drop Gedung B");
            break;
        }
        else{ 
            if(object_position.data[2] != -1){
                centering_B(WP_MISI[MISI][2], "realsense");
                ROS_INFO("Centering di atas...");
            }
            else{
                vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
                goto_wp({current_position.pose.position.x, tfmini_front.range, current_position.pose.position.z});
                ROS_INFO("Gak detek pad...");
	        }
        }
        ros::spinOnce();
        rate.sleep();
    }
    cek_waktu = ros::Time::now();
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
        if (ros::Time::now() - cek_waktu < ros::Duration(1.0)){
            vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
            goto_wp({current_position.pose.position.x, tfmini_front.range, current_position.pose.position.z});
            ROS_INFO("Delay habis drop");
        }
        else if (dist_to_wp(WP_MISI[MISI]) > 0.3 && MISI < 3){
	        ROS_INFO("Balik ke titik gedung...");
            goto_wp(WP_MISI[MISI]);
        }
	    else{
		set_servo(0, 2);
	        break;
	    }
        ros::spinOnce(); 
	    rate.sleep();
    }
    //set_dropper("close");
    cek_waktu = ros::Time::now();
    while(ros::ok() && (current_state.mode == "OFFBOARD" && current_state.armed)){
	if ((ros::Time::now() - cek_waktu < ros::Duration(0.5)) && MISI < 3){
	    ROS_INFO("Delay di titik gedung");
        vel.angular.z   = -mypid_yaw.calculate(default_yaw, current_position.pose.orientation.z);
	    goto_wp({current_position.pose.position.x, tfmini_front.range, current_position.pose.position.z});
	}
    else {
        MISI = MISI + 1;
        CENTERING = false;
        break;
    }
    ros::spinOnce();
    rate.sleep();
    }
}

void Soeromiber::set_servo(int a, int b){
    switch (b){
    case 0:
        servo1_ang.data = a;
        servo2_ang.data = a;
        servo3_ang.data = a;
        servo1_pub.publish(servo1_ang);
        servo2_pub.publish(servo2_ang);
        servo3_pub.publish(servo3_ang);
        break;
    case 1:
        servo1_ang.data = a;
        servo1_pub.publish(servo1_ang);
        break;
    case 2:
        servo2_ang.data = a;
        servo2_pub.publish(servo2_ang);
        break;
    case 3:
        servo3_ang.data = a;
        servo3_pub.publish(servo3_ang);
        break;
    }
}

void Soeromiber::set_dropper(std::string s){
    if (s == "open"){
        if (dropper_status == "CLOSED"){
            dropper_pos.data = 1;
            dropper_pub.publish(dropper_pos);
            dropper_status = "OPENED";
        }
    }
    else if (s == "reset"){
        dropper_pos.data = 0;
        dropper_pub.publish(dropper_pos);
    }
}

bool Soeromiber::is_offboard(){
    if(current_state.mode == "OFFBOARD"){
        return true;
    }
    else{
        return false;
    }
}



void Soeromiber::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Soeromiber::vision_cb(const std_msgs::Int16MultiArray::ConstPtr& msg){
    object_position = *msg;
}

void Soeromiber::localpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

void Soeromiber::tfmini_bottom_cb(const sensor_msgs::Range::ConstPtr& msg){
    tfmini_alt = *msg;
}

void Soeromiber::tfmini_front_cb(const sensor_msgs::Range::ConstPtr& msg){
    tfmini_front = *msg;
}
