/*
 * ===============================================================================
 * pure_pursuit.h
 * Author: Schaefle Tobias
 * Date: 04.03.21
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * A pure pursuit controller for the ackerman drive setup of the SDV (mainly used for
 * collision avoidance with VFH).
 * ===============================================================================
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#endif // PURE_PURSUIT_H

#include <ros/ros.h>

#include <eigen3/Eigen/Core>

#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <algorithm>
#include <mutex>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
#include <ctime>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/math/constants/constants.hpp>

#define POS_THRESHOLD 0.1
#define HEADING_THRESHOLD 30 * MathConstants::PI / 180


namespace MathConstants
{
    constexpr double PI = boost::math::constants::pi<double>();
    constexpr double TWOPI = 2 * boost::math::constants::pi<double>();
    constexpr float PIf = boost::math::constants::pi<float>();
    constexpr float TWOPIf = 2 * boost::math::constants::pi<float>();
    constexpr double EPS = 1e-10;
    constexpr float EPSf = 1e-10f;
}

namespace ControlConstants
{
    constexpr unsigned int HORIZON = 40;    // horizon for MPC
    constexpr double PURE_PURSUIT_LOOKAHEAD = 0.8;
    constexpr double VFH_MAX_GOAL_DISTANCE = 4.5;       // used for full collision avoidance mode
    constexpr double VFH_MIN_GOAL_DISTANCE = 0.8;       // used for full tracking mode
    constexpr double VFH_GOAL_ANGLE = 60 * M_PI / 180;
    constexpr double VFH_DENSITY_CONSTANT = 60;
    constexpr double VFH_MAX_ALPHA = 10 * M_PI / 180;
}

namespace TrajectoryParameters
{
    constexpr double PATH_VEL_LIM  = 0.3;
    constexpr double PATH_VEL_MIN = 0.2;
}

namespace RobotConstants
{

    constexpr double MAX_WHEEL_ACC         = 0.1;              
    constexpr double MAX_WHEEL_VEL         = 0.5;      
    constexpr double AXLE_LENGTH           = 0.53 / 2;             // [m]
    constexpr double MAX_ANG_VEL           = MathConstants::PI/3;  // [rad/s] 
}

namespace GeneralFunctions
{
    inline bool isEqual(const double lhs, const double rhs)
    {
        return std::abs(lhs - rhs) < MathConstants::EPS;
    }

    inline bool isEqualf(const float lhs, const float rhs)
    {
        return std::abs(lhs - rhs) < MathConstants::EPSf;
    }

    inline double wrapTo2Pi(const double rad)
    {
        double value = fmod(rad, MathConstants::TWOPI);
        if (value < 0)
        {
            value += MathConstants::TWOPI;
        }
        return value;
    }

    inline float wrapTo2Pif(const float rad)
    {
        float value = fmodf(rad, MathConstants::TWOPIf);
        if (value < 0)
        {
            value += MathConstants::TWOPIf;
        }
        return value;
    }

    inline double wrapToPi(double rad)
    {
        double value = rad - MathConstants::TWOPI * floor((rad + MathConstants::PI) / MathConstants::TWOPI);
        return value;
    }

    inline float wrapToPif(float rad)
    {
        float value = rad - MathConstants::TWOPIf * floorf((rad + MathConstants::PIf) / MathConstants::TWOPIf);
        return value;
    }

    inline double absoluteDifferenceBetweenAngles(const double rad1, const double rad2)
    {
        double value = MathConstants::PI - fabs(fmod(fabs(rad1 - rad2), MathConstants::TWOPI) - MathConstants::PI);
        return value;
    }

    inline double angularVelocity(const double rad1, const double rad2, const double sampling_time)
    {
        double signed_diff;
        double mod_diff = fmod(fabs(rad1 - rad2), MathConstants::TWOPI);
        if (mod_diff > MathConstants::PI)
        {
            signed_diff = MathConstants::TWOPI - mod_diff;
            if (rad2 > rad1)
            {
                signed_diff *= -1;
            }
        }
        else
        {
            signed_diff = mod_diff;
            if (rad1 > rad2)
            {
                signed_diff *= -1;
            }
        }
        return signed_diff / sampling_time;
    }
    
    template<typename T> static int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

}

class PurePursuit
{
public:
    PurePursuit() {}

    PurePursuit(ros::NodeHandle nh, ros::NodeHandle private_nh,
                double sampling_time)
    {
        sampling_time_ = sampling_time;

        vfh_distance_ = ControlConstants::PURE_PURSUIT_LOOKAHEAD;
        prev_lin_vel_ = 0.0;
        prev_nearest_waypoint_idx_ = 0;

        read_reference();

        time_t now = time(0);
        strftime(filename_, sizeof(filename_), "log/%Y%m%d_%H%M.csv", localtime(&now));

        std::ofstream iniCSV;
        iniCSV.open(filename_, std::ios::out|std::ios::trunc);
       // iniCSV << "Horizon : " + std::to_string(ControlConstants::HORIZON) + ", Velocity" + std::to_string(TrajectoryParameters::PATH_VEL_LIM); 
       // iniCSV << std::endl;
        iniCSV <<   "pose_x [m], pose_y [m], pose_theta [rad], "
                    "v [m/s], delta [rad], "
                    "x_e, y_e, theta_e, x_ref, y_ref, theta_ref";
        iniCSV << std::endl;

    }

    void control(const Eigen::Vector3d &rear_axis_pose);

private:
    // Find nearest waypoint to vehilces currents position
    int findNearestWaypoint(Eigen::Vector2d robot_position);

    void generateCSV(const Eigen::Vector3d &pose, const Eigen::Vector2d &input);
    void read_reference();
    bool atGoal(const Eigen::Vector3d &error) const;

    /// calculate the current velocity with respect to epsilon ("curvature") of the path
    double getLinearVelocity(const Eigen::Vector3d &rear_axis_pose);

    // for generating a trajectory model of the system needed
    inline void getStateUpdate(const Eigen::Vector3d &prev_state, const Eigen::Vector2d input, Eigen::Vector3d &state);

    // get the angle between robot heading and lookahead line
    inline double getAlpha(const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_position);

    // returns the input from the vfh
    void getVfhControlCmd(Eigen::Vector2d &input);

    // check linear acc constraint of ackerman drive for center wheel
    inline double linearAccConstraint(double lin_vel, double alpha)
    {
        double lin_acc = (lin_vel - prev_lin_vel_) / sampling_time_;
        // calculate max acc for center wheel
        double max_acc = RobotConstants::MAX_WHEEL_ACC * vfh_distance_ / (vfh_distance_ + 2 * RobotConstants::AXLE_LENGTH * std::abs(std::sin(alpha)));
        if (std::abs(lin_acc) > max_acc)
        {
            if (lin_acc < 0)
            {
                return prev_lin_vel_ - max_acc * sampling_time_;
            }
            else
            {
                return prev_lin_vel_ + max_acc * sampling_time_;
            }
        }
        return lin_vel;
    }

    // check linear velocity constraint of ackerman drive for center wheel
    inline double linearVelConstraint(double lin_vel, double alpha)
    {
        // calculate max linear velocity for the center wheel
        double max_vel = RobotConstants::MAX_WHEEL_VEL * vfh_distance_ / (vfh_distance_ + 2 * RobotConstants::AXLE_LENGTH * std::abs(std::sin(alpha)));
        if (std::abs(lin_vel) > max_vel)
        {
            if (lin_vel < 0)
            {
                return -1 * max_vel;
            }
            else
            {
                return max_vel;
            }
        }
        return lin_vel;
    }

    char filename_[30];
    
    int flag;

    double sampling_time_;

    double vfh_distance_;

    double prev_lin_vel_;

    Eigen::MatrixXd pose_ref_;
    int prev_nearest_waypoint_idx_;
    Eigen::MatrixXd ref_;

    Eigen::Vector3d goal_;
    Eigen::Vector3d vfh_goal;
    nav_msgs::PathPtr vfh_heading_path;


    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher velo_pub_;
    ros::Timer periodic_timer_;

    ros::Publisher velocity_publisher_;
    geometry_msgs::Twist velocity_msgs_;

};
