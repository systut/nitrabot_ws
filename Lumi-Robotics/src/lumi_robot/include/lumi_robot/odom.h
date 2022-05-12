#ifndef MEDICAL_ROBOT_ODOM_H
#define MEDICAL_ROBOT_ODOM_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class Odom 
{
public:
    Odom();
    void velCallback(const geometry_msgs::Twist& vel);

private:
    // Node handler
	ros::NodeHandle nh_;
    
	// Publisher
	ros::Publisher odom_publisher_;
    
	// Subscriber
	ros::Subscriber velocity_subscriber_;
    
	// Heading quaternion
	tf2::Quaternion odom_quat;
    
	// Odom -> footprint transform
	geometry_msgs::TransformStamped odom_trans;
	
	tf2_ros::TransformBroadcaster odom_broadcaster_;

	nav_msgs::Odometry odom;

    float steering_angle_;
    
	float linear_velocity_x_;
	
	float linear_velocity_y_;
    
	float angular_velocity_z_;
    
	ros::Time last_vel_time_;
    
	float vel_dt_;
	// Robot state
    float x_pos_;
    
	float y_pos_;
    
	float heading_;
};

#endif

