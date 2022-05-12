//下記を記入 --------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string>
#include <cstring>
#include <fstream>
#include <iostream>
#include <tf/tf.h>

// log file

//Get current time and generate file name
std::string getFilenameWithTime()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    
    auto output_s = "/home/syseng402/nitra_ws/src/nitra_robot/data/pose_data_" + s + ".csv"; 
    
    return output_s;
}


std::string filename = getFilenameWithTime();

char const *log_file = filename.c_str();

// open the file for writing
FILE *fp_log = fopen(log_file, "a+");

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
 
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double yaw = tf::getYaw(msg->pose.orientation); 
   
    printf("x (m): %f y(m): %f, heading(rad): %f\n", x, y, yaw);
    fprintf(fp_log, "%f, %f, %f \n", x, y, yaw);
}
 
void encoder_pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
 
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation); 
   
    printf("x (m): %f y(m): %f, heading(rad): %f\n", x, y, yaw);
    fprintf(fp_log, "%f, %f, %f \n", x, y, yaw);
}
 
int main(int argc, char **argv) {

    // open the file for writing
    //FILE *fp_log = fopen(log_file, "a+");
    
    if (fp_log == NULL)
    {
	printf("Error opening the file %s\n", log_file);
	return -1;
    }
    
    fprintf(fp_log, "x(m), y(m), heading(rad) %s \n", "");
    
    ros::init(argc, argv, "pose_point");
    ros::NodeHandle nh;
    ros::Subscriber subscribetf = nh.subscribe("/tracked_pose", 1000, pose_callback);
    ros::spin();
    return(0);
    fclose(fp_log);
}
 //ここまで --------------------------------------------
