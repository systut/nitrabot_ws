#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>


class FeedForwardControl 
{
    private:
        
        ros::Publisher velocity_pub_;
        
        ros::NodeHandle nh_;
        
        // Reference
        Eigen::MatrixXd ref_;
        
        geometry_msgs::Twist velocity_msgs_;
        
        ros::Timer controller_timer_; 

        unsigned long counter_;

        void readReference()
        {
            std::string trajectory_file = "/root/catkin_ws/src/coverage_path_control_input.csv";
            std::ifstream ref_file(trajectory_file.c_str());
            if (ref_file.good())
            {
                // Count lines and initialize reference matrix accordingly
                long num_ref_points = std::count(std::istreambuf_iterator<char>(ref_file),
                                                 std::istreambuf_iterator<char>(), '\n');
                                 
                // first line contains column headers
                --num_ref_points;
                ref_ = Eigen::MatrixXd(3, num_ref_points);

                // Clear bad state from counting lines and reset iterator
                ref_file.clear();
                ref_file.seekg(0);

                std::string line;
                std::getline(ref_file,line);    // read first line (column headers)
                
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

        void execute(const ros::TimerEvent& event)
        {
            std::cout << "Step " << counter_ << " :" << std::endl;
            
            if (counter_ == 1400)
            {
                velocity_msgs_.linear.x = 0.0;
                
                velocity_msgs_.angular.z = 0.0;
                
                velocity_pub_.publish(velocity_msgs_);

                ros::shutdown();
            }
             
            Eigen::Vector2d input_ref = ref_.block(1, counter_, 2, 1);
            
            std::cout << input_ref << std::endl;
            
            velocity_msgs_.linear.x = input_ref(0);
            
            velocity_msgs_.angular.z = input_ref(1);

            ++counter_;

            velocity_pub_.publish(velocity_msgs_);
        }
    
    public:
        FeedForwardControl(ros::NodeHandle nh)
        {
            nh_ = nh;
            
            readReference();
            
            counter_ = 0;


            controller_timer_ = nh_.createTimer(ros::Duration(0.1), &FeedForwardControl::execute, this);

            velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feedforward_controller");

    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(0);

    spinner.start();
    
    FeedForwardControl controller(nh);
    
    ros::waitForShutdown();
}


