
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


namespace ftc_local_planner
{

    class FTCPlanner : public nav_core::BaseLocalPlanner
    {

    private:
        void reconfigureCB(FTCPlannerConfig &config, uint32_t level);

        dynamic_reconfigure::Server<FTCPlannerConfig> *reconfig_server;

        tf2_ros::Buffer* tf_buffer;
        costmap_2d::Costmap2DROS* costmap;
        std::vector<geometry_msgs::PoseStamped> global_plan;

        ros::Publisher global_point_pub;
        ros::Publisher local_point_pub;

        ftc_local_planner::FTCPlannerConfig config;


        Eigen::Affine3d current_control_point;

        uint32_t current_index;
        double current_progress;


        ros::Time last_time;

        bool finished = false;

        double last_distance_error = 0.0;
        double last_angle_error = 0.0;

        bool wait_for_min_dist = false;

        bool rotate_mode = false;
        double rotation_target_angle;
        double dt;



        void moveControlPoint();

        bool computeVelocityCommandsFollow(geometry_msgs::Twist &cmd_vel);
        bool computeVelocityCommandsRotate(geometry_msgs::Twist &cmd_vel);

    public:
        FTCPlanner();

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

        bool isGoalReached() override;

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

        ~FTCPlanner() override;


    };
};
#endif
