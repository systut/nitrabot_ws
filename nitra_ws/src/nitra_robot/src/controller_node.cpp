/*
 * ===============================================================================
 * controller_node.cpp
 * Author: Tobias Schaefle
 * Date: 27.06.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This is the main node for the contrller of the SDV. 
 * It receives information from the task manager and also information of the robot pose 
 * and the current encoder values. It calls the controller in a fixed sampling time.
 * ===============================================================================
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>

#include <boost/make_shared.hpp>

// #include "../include/mpc_ackerman.h"
#include "../include/pure_pursuit.h"

#include <cmath>
#include <mutex>

class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        init(nh, private_nh);
        std::cout << "Init succesfull" << std::endl;
        initController();
        std::cout << "Controller init succesfull" << std::endl;
        ros::Duration(1).sleep();
    }

    void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        double now = ros::Time::now().toSec();
        std::cout << "Call time odom:\t" << now << "sec" << std::endl;
        // std::cout << odom_msg->pose.pose.position.x << std::endl;
        // std::cout << "Call Odom" << std::endl;
        // store robot pose
        odom_msg_ = odom_msg;
    }

    void callController(const ros::TimerEvent &event)
    {
        double now = ros::Time::now().toSec();
        //std::cout << "Call time:\t" << last_ - now << "sec" << std::endl;
        last_ = now;
        //std::cout << "Call Controller" << std::endl;
        // lock the data
        std::lock_guard<std::mutex> lock_encoder(encoder_mutex_);


        // check if ok to execute
        //if (collision_msg_ == nullptr || encoder_msg_ == nullptr || odom_msg_ == nullptr)
        if (odom_msg_ == nullptr)
        {
            ROS_WARN("CONTROLLER_NODE_WARN: No messages received from odom or encoder.");
            return;
        }

        // take data
        // pose
        double yaw = tf::getYaw(odom_msg_->pose.orientation);
        if (yaw < 0)
        {yaw += 2.0 * M_PI;}
        robot_pose_ << odom_msg_->pose.position.x, odom_msg_->pose.position.y, yaw;
        //std::cout << "yaw:" << yaw << std::endl;
        odom_mutex_.unlock();


        std::cout << "Follow Trajectory" << std::endl;
        // mpc_controller_.control(robot_pose_);
        pure_pursuit_controller_.control(robot_pose_);


        std::cout << "Function call time:\t" << ros::Time::now().toSec() - now << "sec" << std::endl;
    }

private:
    ros::NodeHandle nh_, private_nh_;
    // subscriber to robot localization and encoder data
    ros::Subscriber odom_sub_;

    // vector of robot pose (x, y, theta)
    // messages
    geometry_msgs::PoseStamped::ConstPtr odom_msg_;
    nav_msgs::PathPtr vfh_path_;

    bool simulation_;

    // steer is in rad and wheel_vel is in m/s
    Eigen::Vector3d robot_pose_;

    // sampling time of the node
    double sampling_time_;
    ros::Timer periodic_timer_;
    // mutex for all variables which are affected of the multiple threads
    std::mutex odom_mutex_, encoder_mutex_, service_mutex_, collision_mutex_;
    // object of the controller
    // MpcAckerman mpc_controller_;
    PurePursuit pure_pursuit_controller_;

    // service object
    ros::ServiceClient flag_traj_client_;

    int update_counter_;

    // timer
    double last_;

    void init(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        nh_ = nh;
        private_nh_ = private_nh;
        // check parameter server
        nh.param("controller_sampling_time", sampling_time_, 0.05); //[s]
        nh.param("simulation", simulation_, true);
        // subscriber
        ROS_WARN("CONTROLLER_NODE_WARN: Subscribe 'odom' and 'encoder_odom' conditions");
        //odom_sub_ = nh_.subscribe("/encoder_odom", 4, &ControllerNode::odometryCallback, this);
        odom_sub_ = nh_.subscribe("/tracked_pose", 4, &ControllerNode::odometryCallback, this);
        periodic_timer_ = private_nh_.createTimer(ros::Duration(sampling_time_), &ControllerNode::callController, this);
    }

    void initController()
    {
        // mpc_controller_ = MpcAckerman(nh_, private_nh_, sampling_time_);
        pure_pursuit_controller_ = PurePursuit(nh_, private_nh_, sampling_time_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ControllerNode controller(nh, private_nh);

    ros::AsyncSpinner s(4);
    s.start();
    ros::waitForShutdown();

    return 0;
}
