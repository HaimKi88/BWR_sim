#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <iostream>

struct GPS {
    double lat, lon;
};

struct Quaternion {
    double w, x, y, z;
};

class Tools{
    public:
    static double deg2rad(double deg){
        return deg * M_PI / 180.0;
    }
    static double rad2deg(double deg){
        return deg * 180.0 / M_PI;
    }
};

class CarController{
    public:

    GPS robo_loc, trgt_loc;
    geometry_msgs::Twist twist_msg;

    double heading;
    double heading_error;
    double trgtAng;
    double dist_error;
    double steer_cmd;
    double vel_cmd;
    bool arrived = true;          // flag 
    double I = 0;                 // Integral controller sum
    double last_err = 0;          // last error for D control

    const double R = 6371000.0;   // Earth's radius in meters
    const double max_vel = 4;
    const double max_steer = 30 * M_PI / 180;
    const double target_tolerance = 1;  // Tolerance for considering the target reached
    const double rate = 10.0;     

    ros::Subscriber gps_sub;
    ros::Subscriber target_sub;
    ros::Subscriber imu_sub; 
    ros::Publisher  cmd_vel_pub;

    CarController(){
        ros::NodeHandle n;
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        gps_sub    = n.subscribe("/robot_location", 1000, &CarController::locationCallback, this);
        target_sub = n.subscribe("/target_location", 1000, &CarController::targetCallback, this);
        imu_sub    = n.subscribe("/imu", 1000, &CarController::imuCallback, this);
    }

    void publishMsg(){
        cmd_vel_pub.publish(twist_msg);
    }
    
    double calcHeadingErr(){
        return targetAngle(trgt_loc,robo_loc) - heading;
    }

    double targetAngle(GPS p1, GPS p2) {
        // Convert latitude and longitude from degrees to radians
        double lat1 = Tools::deg2rad(p1.lat);
        double lon1 = Tools::deg2rad(p1.lon);
        double lat2 = Tools::deg2rad(p2.lat);
        double lon2 = Tools::deg2rad(p2.lon);

        // Calculate the differences in coordinates
        double dLon = lon2 - lon1;

        // Calculate the heading angle using the haversine formula
        double y = sin(dLon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
        double angle = atan2(y, x);
        
        // A rotated axis system (left and right)
        if (angle < 0){
            angle = -(angle+M_PI);
        } else if (angle > 0){
            angle = -(angle-M_PI);
        }
        return angle;
    }

    double quaternionToYaw(const Quaternion& q) {
        return atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        Quaternion quaternion = {msg->orientation.x, 
                                msg->orientation.y, 
                                msg->orientation.z, 
                                msg->orientation.w};

        heading = quaternionToYaw(quaternion);
    }

    void targetCallback(const geographic_msgs::GeoPoint::ConstPtr& msg){
        trgt_loc.lat = msg->latitude; 
        trgt_loc.lon = msg->longitude;
        arrived = false;
        ROS_INFO("Received target Location - Latitude: %f, Longitude: %f", msg->latitude, msg->longitude);
    }

    void locationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        robo_loc.lat = msg->latitude; 
        robo_loc.lon = msg->longitude;
    }

    double haversine(const GPS& point1, const GPS& point2) {
        // Convert latitude and longitude from degrees to radians
        double lat1 = Tools::deg2rad(point1.lat);
        double lon1 = Tools::deg2rad(point1.lon);
        double lat2 = Tools::deg2rad(point2.lat);
        double lon2 = Tools::deg2rad(point2.lon);

        // Calculate the differences in coordinates
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        // Haversine formula
        double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
                cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
        double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

        // Distance in meters
        double distance = R * c;

        return distance;
    }

    double velocityControl(double err, double heading_err){
        if (err < target_tolerance){
            arrived = true;
            return 0;
        }
        else {
            if (heading_err < Tools::deg2rad(15)){
                return std::min(err, max_vel);
            }
            else{
                return std::min(err, max_vel/2);
            }
        }
    }

    double velocityControl(double err) {
        double kp;
        double kd = 0.1;
        double ki = 0.1;
        
        // adaptive P
        if (dist_error < 60) { kp = 0.2; }
        else { kp = 0.32; }
        // clear I if needed
        if (err < abs(Tools::deg2rad(5))){
            I = I + ki*err*(1/rate);
        } else { I = 0; }

        // calc PID
        double P = err*kp;
        double D = kd*(err-last_err)/(1/rate);
        double cmd = P + D  + I;
       
        last_err = err;
        // Limit command
        if (cmd > 0){
            return std::min(cmd,max_steer);
        }
        else if (cmd < 0){
            return std::max(cmd,-max_steer);
        }
        else {
            return cmd;
        }
    }

    void printMsg(){
        if (not arrived){
            ROS_INFO("heading err:   %.1f, dist error: %.1f ", Tools::rad2deg(heading_error), dist_error);
            ROS_INFO("steer cmd:     %.1f, vel cmd:    %.1f", Tools::rad2deg(steer_cmd), vel_cmd);
            ROS_INFO("target angle:  %.1f, heading:    %.1f", Tools::rad2deg(targetAngle(trgt_loc,robo_loc)),Tools::rad2deg(heading));
            ROS_INFO(" ");
        }
    }

    void controlStep(){
        heading_error = calcHeadingErr();
        dist_error    = haversine(robo_loc, trgt_loc);
        
        steer_cmd = velocityControl(heading_error);
        vel_cmd = velocityControl(dist_error,heading_error);
        
        if (arrived){
            vel_cmd = 0;
            steer_cmd = 0;
            I = 0;
            ROS_INFO("car arrived, distance to target: %.1f", dist_error);
        }

        twist_msg.linear.x = vel_cmd;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = steer_cmd;

        publishMsg();
        printMsg();        
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    CarController controller;
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        controller.controlStep();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

