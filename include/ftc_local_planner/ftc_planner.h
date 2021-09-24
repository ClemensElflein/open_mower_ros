
#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <ftc_local_planner/FTCPlannerConfig.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include "tf2_eigen/tf2_eigen.h"
#include <mbf_costmap_core/costmap_controller.h>


namespace ftc_local_planner {

class FTCPlanner : public mbf_costmap_core::CostmapController {

    private:
        void reconfigureCB(FTCPlannerConfig &config, uint32_t level);

        dynamic_reconfigure::Server<FTCPlannerConfig> *reconfig_server;

        tf2_ros::Buffer *tf_buffer;
        costmap_2d::Costmap2DROS *costmap;
        std::vector<geometry_msgs::PoseStamped> global_plan;

        ros::Publisher global_point_pub;
        ros::Publisher local_point_pub;
        ros::Publisher global_plan_pub;

        ftc_local_planner::FTCPlannerConfig config;


        Eigen::Affine3d current_control_point;

        uint32_t current_index;
        double current_progress;


        ros::Time last_time;

        int planner_step = 0;
        bool goal_reached = false;
        ros::Time planner_position_time;
        ros::Time planner_rotation_time;

        double last_lon_error = 0.0;
        double last_lat_error = 0.0;
        double last_angle_error = 0.0;

        double i_lon_error = 0.0;
        double i_lat_error = 0.0;
        double i_angle_error = 0.0;

        double dt;

        double current_movement_speed;
        bool crashed;


        int moveControlPoint();

        uint32_t computeVelocityCommandsFollow(geometry_msgs::Twist &cmd_vel);
        double distanceLookahead();

    public:
        FTCPlanner();


        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

        ~FTCPlanner() override;

    uint32_t
    computeVelocityCommands(const geometry_msgs::PoseStamped &pose, const geometry_msgs::TwistStamped &velocity,
                            geometry_msgs::TwistStamped &cmd_vel, std::string &message) override;

    bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

    bool cancel() override;


};
};
#endif
