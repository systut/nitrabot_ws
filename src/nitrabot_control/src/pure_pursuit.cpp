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

#include "../include/pure_pursuit.h"

void PurePursuit::read_reference()
{
    std::ifstream ref_file("/root/catkin_ws/src/pure_pursuit_trajectory.csv");

    //Debug for error opening file
    if (!ref_file) {
        std::cout << "Error opening file"<<std::endl;
    }

    // Count lines and initialize reference matrix accordingly
    long num_ref_points = std::count(std::istreambuf_iterator<char>(ref_file),
                                        std::istreambuf_iterator<char>(), '\n');
                                        
    // first line contains column headers
    --num_ref_points;

    ref_ = Eigen::MatrixXd(8, num_ref_points);

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

    ref_file.close();

    
    // pose_ref_ = ref_.block(1, 1, 3, num_ref_points-1);

    vfh_goal = ref_.block(1, num_ref_points-1, 3, 1);
    // vfh_heading_path = ;
    // v_r, v_l
    // w,v
}

double PurePursuit::getLinearVelocity(const Eigen::Vector3d &rear_axis_pose)
{
    // get distance vector
    Eigen::Vector2d distance = (vfh_goal.head<2>() - rear_axis_pose.head<2>()).normalized();
    // normal vector depends on turn direction
    double angle_change = GeneralFunctions::wrapToPi(GeneralFunctions::wrapTo2Pi(vfh_goal.z()) - GeneralFunctions::wrapTo2Pi(rear_axis_pose.z()));
    double theta_shift = vfh_goal.z() + M_PI_2 * GeneralFunctions::sgn<double>(angle_change);
    Eigen::Vector2d normal_vector(std::cos(theta_shift), std::sin(theta_shift));

    // get epsilon
    double epsilon = distance.dot(normal_vector);
    // std::cout << "Epsilon: " << std::abs(epsilon) << std::endl;
    // calculate velocity
    double lin_vel = TrajectoryParameters::PATH_VEL_LIM + std::abs(epsilon) * (TrajectoryParameters::PATH_VEL_MIN - TrajectoryParameters::PATH_VEL_LIM);
    // std::cout << "Velocity: " << lin_vel << std::endl;
    return lin_vel;
}

void PurePursuit::control(const Eigen::Vector3d &rear_axis_pose)
{
    // calculate error
    Eigen::Vector3d error = vfh_goal - rear_axis_pose;
    double angle_error = GeneralFunctions::wrapToPi(GeneralFunctions::wrapTo2Pi(vfh_goal.z()) - GeneralFunctions::wrapTo2Pi(rear_axis_pose.z()));
    error(2) = angle_error;

    std::cout << "Error:\n" << error << std::endl;
    // check if at goal
    // if (atGoal(error))
    // {
    //     std::cout << "At goal" << std::endl;
    //     // stopMotion();
    //     return;
    // }

    // bool pub_path = false;
    // if (vfh_path_pub_.getNumSubscribers() > 0)
    // {
    //     vfh_pp_path_msg_.poses.clear();
    //     vfh_pp_path_msg_.poses.resize(10 * ControlConstants::HORIZON);
    //     pub_path = true;
    // }
    std::cout << "goal:\n" << vfh_goal << std::endl;
    Eigen::Vector2d robot_position = rear_axis_pose.head<2>();
    Eigen::Vector2d vfh_position;
    double distance, alpha, ang_vel;
    double lin_vel_pp = getLinearVelocity(rear_axis_pose);

    double lin_vel = lin_vel_pp;

    int nearest_waypoint_idx, lookahead_idx;
    nearest_waypoint_idx = findNearestWaypoint(robot_position);

    for (long ii = nearest_waypoint_idx; ii < ref_.cols(); ++ii)
    {
        vfh_position = ref_.block(1, ii, 2, 1);
        distance = (vfh_position - robot_position).norm();
 
        if (distance > vfh_distance_)
        {
           lookahead_idx = ii -1; 
           vfh_position = ref_.block(1, lookahead_idx, 2, 1);
           distance = (vfh_position - robot_position).norm();
           break;
        }
    }

    // calcuate angle
    alpha = getAlpha(rear_axis_pose, vfh_position);

    // calculate constraints for linear velocity
    lin_vel = linearVelConstraint(lin_vel, alpha);
    lin_vel = linearAccConstraint(lin_vel, alpha);

    prev_lin_vel_ = lin_vel;

    // calculate angular velocity
    ang_vel = 2 * lin_vel * std::sin(alpha) / distance;
    
    std::cout << "computed ouput: (v,w) = " << lin_vel << " , " << ang_vel << std::endl;

    if (flag == 1)
    {
        lin_vel = 0.0;
        ang_vel = 0.0;
    }
    else if(std::abs(ang_vel)> RobotConstants::MAX_ANG_VEL)
    {
      std::cout << "[Warn] Command angular velocity excesses the maximum value ! Force to stop !" << std::endl;  
      lin_vel = 0.0;
      ang_vel = 0.0;
      flag =1;
    }
    else if(atGoal(error))
    {
        std::cout << "[Info] At goal" << std::endl;
        lin_vel = 0.0;
        ang_vel = 0.0;
        flag = 1;
    }

    std::cout << " the Output: (v,w) = " << lin_vel << " , " << ang_vel << std::endl;

    // calculate actions for SDV
    Eigen::Vector2d input(lin_vel, ang_vel);
    getVfhControlCmd(input);

    generateCSV(rear_axis_pose, input);
}

bool PurePursuit::atGoal(const Eigen::Vector3d &error) const
{   

    if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD && std::abs(error(2)) < HEADING_THRESHOLD)
    {
        return true;
    }
    return false;
}

inline void PurePursuit::getStateUpdate(const Eigen::Vector3d &prev_state, const Eigen::Vector2d input, Eigen::Vector3d &state)
{
    if (GeneralFunctions::isEqual(input.y(), 0.0))
    {
        state << prev_state.x() + input.x() * sampling_time_ * std::cos(prev_state.z()),
                 prev_state.y() + input.y() * sampling_time_ * std::sin(prev_state.z()),
                 prev_state.z() + 0.0;
    }
    else
    {
        double radius = input.x() / input.y();
        state << prev_state.x() - radius * std::sin(prev_state.z()) + radius * std::sin(prev_state.z() + input.y() * sampling_time_),
                 prev_state.y() + radius * std::cos(prev_state.z()) - radius * std::cos(prev_state.z() + input.y() * sampling_time_),
                 GeneralFunctions::wrapTo2Pi(prev_state.z() + input.y() * sampling_time_);
    }
}

inline double PurePursuit::getAlpha(const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_position)
{
    Eigen::Vector2d vec1(std::cos(robot_pose.z()), std::sin(robot_pose.z()));
    Eigen::Vector2d vec2 = vfh_position - robot_pose.head<2>();

    // "https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/issues/index.htm"
    double angle = std::atan2(vec2(1), vec2(0)) - std::atan2(vec1(1), vec1(0));
    if (angle > M_PI){angle -= 2 * M_PI;}
    else if (angle < -M_PI) {angle += 2 * M_PI;}
    //std::cout << "Angle:\t" << angle << std::endl;
    return angle;
}

void PurePursuit::getVfhControlCmd(Eigen::Vector2d &input)
{
    double angular_vel_ref = input(1);
    double linear_vel_ref = input(0);

    if (std::abs(angular_vel_ref) < 0.001)
    {
        double angular_vel_ref = 0.0;

    }

    velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    velocity_msgs_.linear.x = linear_vel_ref;
    velocity_msgs_.angular.z = angular_vel_ref;
    velocity_publisher_.publish(velocity_msgs_);
}

void PurePursuit::generateCSV(const Eigen::Vector3d &pose, const Eigen::Vector2d &input)
{
    std::ofstream export_data;

    export_data.open(filename_, std::ios::out|std::ios::app);
    for (long ii = 0; ii < pose.rows(); ++ii)
    {
        export_data << pose(ii) << ", ";
    }
    for (long ii = 0; ii < input.rows(); ++ii)
    {
        export_data << input(ii);
        if (ii != (input.rows()-1))
        {
            export_data << ", ";
        }
    }
  //  for (long ii = 0; ii < error.rows(); ++ii)
  //  {
  //      export_data << error(ii) << ", ";
  //  }
  //  for (long ii = 0; ii < state_ref.rows(); ++ii)
  //  {
  //      export_data << state_ref(ii);
  //      if (ii != (state_ref.rows()-1))
  //      {
  //          export_data << ", ";
  //      }
  //  }
    export_data << std::endl;
}

int PurePursuit::findNearestWaypoint(Eigen::Vector2d robot_position)
{
    //Reduce search space T
    //TODO: Find a systematic way to select value 5
    int lb_idx = std::max(0, prev_nearest_waypoint_idx_);
    int last_idx = static_cast<int>(ref_.cols())-1;
    int ub_idx = std::min(last_idx, prev_nearest_waypoint_idx_ + 5);

    int nearest_waypoint_idx;
    float smallest_distance = std::numeric_limits<float>::max();

    for (int ii = lb_idx; ii <= ub_idx; ii++)
    {
        float distance = (ref_.block(1, ii, 2, 1) - robot_position).norm();

        if (distance < smallest_distance)
        {
            nearest_waypoint_idx = ii;
            smallest_distance = distance;
        }
    }

    prev_nearest_waypoint_idx_ = nearest_waypoint_idx;

    return nearest_waypoint_idx;
}