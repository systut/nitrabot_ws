#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

class ImuDebugger
{   
    private:
        ros::Publisher imu_pose_pub_;
        
        ros::Subscriber imu_sub_;

        bool debug_ = true;

        const double RAD_TO_DEG_ = 180/3.14159;

        void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
        {
            geometry_msgs::PoseStamped pose_msg;

            pose_msg.header = imu_msg->header;
            pose_msg.pose.orientation = imu_msg->orientation;

            imu_pose_pub_.publish(pose_msg);

            if (debug_== true)
            {
                // Convert quaternion to rpy
                tf::Quaternion quat(
                    imu_msg->orientation.x,
                    imu_msg->orientation.y,
                    imu_msg->orientation.z,
                    imu_msg->orientation.w);
                
                tf::Matrix3x3 m(quat);

                double roll, pitch, yaw;

                m.getRPY(roll, pitch, yaw);
                

                ROS_WARN_STREAM("rpy = " << roll*RAD_TO_DEG_ <<", " << pitch*RAD_TO_DEG_ << ", " << yaw*RAD_TO_DEG_);
            }
            
        }

    public:
        ImuDebugger(ros::NodeHandle& nh)
        {
            imu_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("imu/pose", 10);
            imu_sub_ = nh.subscribe("imu/data", 10, &ImuDebugger::imuCallback, this);
        }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "img_debug_node");
    
    ros::NodeHandle nh;
    
    ImuDebugger imu_debugger(nh);

    ros::spin();

    return 0;
}