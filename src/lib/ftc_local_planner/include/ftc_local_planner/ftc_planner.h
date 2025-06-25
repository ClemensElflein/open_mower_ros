
#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_H_

#include <ros/ros.h>
#include "ftc_local_planner/PlannerGetProgress.h"
#include "ftc_local_planner/oscillation_detector.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <ftc_local_planner/FTCPlannerConfig.h>
#include <ftc_local_planner/PID.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include "tf2_eigen/tf2_eigen.h"
#include <mbf_costmap_core/costmap_controller.h>
#include <visualization_msgs/Marker.h>

namespace ftc_local_planner
{

    class FTCPlanner : public mbf_costmap_core::CostmapController
    {

        enum PlannerState
        {
            PRE_ROTATE,
            FOLLOWING,
            WAITING_FOR_GOAL_APPROACH,
            POST_ROTATE,
            FINISHED
        };

    private:
        ros::ServiceServer progress_server;
        // State tracking
        PlannerState current_state;
        ros::Time state_entered_time;

        bool is_crashed;

        dynamic_reconfigure::Server<FTCPlannerConfig> *reconfig_server;

        tf2_ros::Buffer *tf_buffer;
        costmap_2d::Costmap2DROS *costmap;
        costmap_2d::Costmap2D* costmap_map_;   

        std::vector<geometry_msgs::PoseStamped> global_plan;
        ros::Publisher global_point_pub;
        ros::Publisher global_plan_pub;
        ros::Publisher progress_pub;
        ros::Publisher obstacle_marker_pub;

        FTCPlannerConfig config;

        Eigen::Affine3d current_control_point;

        /**
         * PID State
         */
        double lat_error, lon_error, angle_error = 0.0;
        double last_lon_error = 0.0;
        double last_lat_error = 0.0;
        double last_angle_error = 0.0;
        double i_lon_error = 0.0;
        double i_lat_error = 0.0;
        double i_angle_error = 0.0;
        ros::Time last_time;

        /**
         * Speed ramp for acceleration and deceleration
         */
        double current_movement_speed;

        /**
         * State for point interpolation
         */
        uint32_t current_index;
        double current_progress;
        Eigen::Affine3d local_control_point;

        /**
         * Private members
         */
        ros::Publisher pubPid;
        FailureDetector failure_detector_; //!< Detect if the robot got stucked
        ros::Time time_last_oscillation_;  //!< Store at which time stamp the last oscillation was detected
        bool oscillation_detected_ = false;
        bool oscillation_warning_ = false;

        double distanceLookahead();
        PlannerState update_planner_state();
        void update_control_point(double dt);
        void calculate_velocity_commands(double dt, geometry_msgs::TwistStamped &cmd_vel);

        /**
         * @brief check for obstacles in path as well as collision at actual pose
         * @param max_points number of path segments (of global path) to check
         * @return true if collision will happen.
         */
        bool checkCollision(int max_points);

        /**
         * @brief check if robot oscillates (only angular). Can be used to do some recovery
         * @param cmd_vel last velocity message send to robot
         * @return true if robot oscillates
         */
        bool checkOscillation(geometry_msgs::TwistStamped &cmd_vel);

        /**
         * @brief publish obstacles on path as marker array.
         * @brief If obstacle_points contains more elements than maxID, marker gets published and
         * @brief cleared afterwards.
         * @param obstacle_points already collected points to visualize
         * @param x X position in costmap
         * @param y Y position in costmap
         * @param cost cost value of cell
         * @param maxIDs num of markers before publishing
         * @return sum of `values`, or 0.0 if `values` is empty.
         */
        void debugObstacle(visualization_msgs::Marker &obstacle_points, double x, double y, unsigned char cost, int maxIDs);

        double time_in_current_state()
        {
            return (ros::Time::now() - state_entered_time).toSec();
        }

        void reconfigureCB(FTCPlannerConfig &config, uint32_t level);

    public:
        FTCPlanner();

        bool getProgress(ftc_local_planner::PlannerGetProgressRequest &req, ftc_local_planner::PlannerGetProgressResponse &res);

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
