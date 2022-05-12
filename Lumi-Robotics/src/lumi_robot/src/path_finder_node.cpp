#include <algorithm>
#include <cmath>
#include <fstream>
#include <tf/tf.h>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#define PI 3.1415

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum PathFinderDirection
{
    FORWARD,
    BACKWARD
};


class PathFinderNode
{
    private:
        ros::NodeHandle nh_;

        // Reference
        Eigen::MatrixXd ref_;

        geometry_msgs::Twist velocity_msgs_;

        ros::Publisher velocity_publisher_;
        ros::Subscriber sub; 
        double x_, y, yaw_;

        unsigned long counter_;
        
        void readReference(std::string trajectory_file)
        { 
            std::ifstream ref_file(trajectory_file.c_str());
            if (ref_file.good())
            {
                // Count lines and initialize reference matrix accordingly
                long num_ref_points = std::count(std::istreambuf_iterator<char>(ref_file),
                                                 std::istreambuf_iterator<char>(), '\n');
                // first line contains column headers
                ref_ = Eigen::MatrixXd(3, num_ref_points);

                // Clear bad state from counting lines and reset iterator
                ref_file.clear();
                ref_file.seekg(0);

                std::string line;
                // std::getline(ref_file,line);    // read first line (column headers)
                // std::cout << line;
                long col_idx = 0;
                while(std::getline(ref_file, line))
                {
                    std::stringstream lineStream(line);
                    std::string cell;
                    long row_idx = 0;
                    while(std::getline(lineStream, cell, ','))
                    {
                        ref_(row_idx,col_idx) = std::stof(cell);
                        ++row_idx;
                    }
                    ++col_idx;
                }
            }
            ref_file.close();
        }


        move_base_msgs::MoveBaseGoal createGoal(unsigned int trajectory_index)
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = ref_(0, trajectory_index);
            ROS_INFO("x:%f", goal.target_pose.pose.position.x);
            goal.target_pose.pose.position.y = ref_(1, trajectory_index);
            ROS_INFO("y:%f", goal.target_pose.pose.position.y);
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, ref_(2, trajectory_index));    
            q = q.normalize();
            goal.target_pose.pose.orientation.x = q.x();
            goal.target_pose.pose.orientation.y = q.y();
            goal.target_pose.pose.orientation.z = q.z();
            goal.target_pose.pose.orientation.w = q.w();
            //goal.target_pose.pose.orientation.w = ref_(2, trajectory_index);
            ROS_INFO("heading: %f", goal.target_pose.pose.orientation.w);  
            return goal;
        }
        

    public:
        PathFinderNode(ros::NodeHandle nh)
        {
            nh_ = nh;

            velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
            
            sub = nh.subscribe("tracked_pose", 10, &PathFinderNode::odomCallback, this);

     	}
        void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
        {
            tf::Quaternion q(
                pose->pose.orientation.x,
                pose->pose.orientation.y,
                pose->pose.orientation.z,
                pose->pose.orientation.w
            );

            tf::Matrix3x3 m(q);
            
            double roll, pitch, yaw;

            m.getRPY(roll, pitch, yaw);

            yaw_ = yaw;
            x_ = pose->pose.position.x;
        }
        
        void move(PathFinderDirection direction)
        {
            std::string trajectory_file;  
            if (direction == FORWARD)
            {
                trajectory_file = "/home/ubuntu/Lumi-Robotics/src/lumi_robot/data/forward_trajectory.csv";
            }
            else if (direction == BACKWARD)
            {
                trajectory_file = "/home/ubuntu/Lumi-Robotics/src/lumi_robot/data/backward_trajectory.csv";
            }
            readReference(trajectory_file);
                
            MoveBaseClient action_client("move_base", true);
            while(!action_client.waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for the move_base action server to come up");
            }
            //std::cout << ref_; 
            std::cout << ref_.cols(); 
            std::cout << ref_.rows(); 
            for (unsigned int trajectory_index = 0; trajectory_index < ref_.rows(); trajectory_index++)
            {
                move_base_msgs::MoveBaseGoal goal;
                goal = createGoal(trajectory_index);
                ROS_INFO("Sending goal : %f", goal);
                action_client.sendGoal(goal);
                ROS_INFO("Waiting for goal to send");
                action_client.waitForResult();
                ROS_INFO("start moving");
                if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("Hooray, the base moved 1 meter forward");
                else
                    ROS_INFO("The base failed to move forward 1 meter for some reason");

            }
        }

        void rotate(double targetAngle)        
        {
            ros::param::get("/targetAngle", targetAngle);

            geometry_msgs::Twist velocity_msg;

            ros::Rate controller_rate(10);
            
            double rotate_Kp = 0.55;            
        
            ros::param::get("/rotate_Kp", rotate_Kp);

            double rotate_stopCondition = 0.4;
            
            ros::param::get("/rotate_stopCondition", rotate_stopCondition);
    
            while ( std::abs(targetAngle*PI/180 - yaw_) > rotate_stopCondition) 
            {
                velocity_msg.angular.z = rotate_Kp * (targetAngle*PI/180 - yaw_); 
                
                velocity_publisher_.publish(velocity_msg);
                
                std::cout << "current :" << yaw_ << ", target: " << targetAngle*PI/180;
                
                ros::spinOnce();
                
                controller_rate.sleep();
            }
            
        }
        
        void parking(double parkingDistance, int directionInMap)

        {
            ros::param::get("/parkingDistance", parkingDistance);

            ros::param::get("/directionInMap", directionInMap);
        
            double targetX = x_ + parkingDistance; 
            
            ros::param::get("/targetX", targetX);
    
            geometry_msgs::Twist velocity_msg;

            ros::Rate controller_rate(10);
            
            double parking_Kp = 0.5;                     
            
            double parking_stopCondition = 0.01;
            
            ros::param::get("/parking_Kp", parking_Kp);

            ros::param::get("/stopCondition", parking_stopCondition);
 
            while ( std::abs(targetX - x_) > parking_stopCondition ) 
            {
                velocity_msg.linear.x =  directionInMap *parking_Kp * (targetX - x_); 
                
                velocity_publisher_.publish(velocity_msg);
                
                ros::spinOnce();
                
                std::cout << "current :" << x_ << ", target: " << targetX;
                
                controller_rate.sleep();
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_finder_node");

    ros::NodeHandle nh;

    PathFinderNode path_finder(nh);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    
    spinner.start(); 

    std::string mode;

    ros::param::get("/mode", mode);
    
    if (mode == "forward")
    {
        path_finder.move(FORWARD);

        ROS_INFO("Move forward DONE");    
     
        path_finder.rotate(-180);

        ROS_INFO("Rotate DONE");

        path_finder.parking(0.2, -1); 
    }   
    else if (mode == "backward")
    {
        path_finder.move(BACKWARD);
        
        ROS_INFO("Move backward DONE");

        path_finder.rotate(0);

        path_finder.parking(-0.2, 1);
        
        ROS_INFO("STOP");

    }
    
    ros::shutdown();
     
    return 0;
}


