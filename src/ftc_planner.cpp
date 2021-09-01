
#include <ftc_local_planner/ftc_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, nav_core::BaseLocalPlanner)

namespace ftc_local_planner {

    FTCPlanner::FTCPlanner() {
    }

    void FTCPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) {
        ros::NodeHandle private_nh("~/" + name);
        local_point_pub = private_nh.advertise<geometry_msgs::PoseStamped>("local_point", 1);
        global_point_pub = private_nh.advertise<geometry_msgs::PoseStamped>("global_point", 1);

        costmap = costmap_ros;
        tf_buffer = tf;

        //Parameter for dynamic reconfigure
        reconfig_server = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this,
                                                                                     _1, _2);
        reconfig_server->setCallback(cb);

        finished = false;


        ROS_INFO("FTCPlanner: Version 2 Init.");
    }

    void FTCPlanner::reconfigureCB(FTCPlannerConfig &c, uint32_t level) {
        if (c.restore_defaults) {
            reconfig_server->getConfigDefault(c);
            c.restore_defaults = false;
        }
        config = c;
    }

    bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
        finished = false;
        global_plan = plan;
        current_index = 0;
        current_progress = 0.0;
        last_time_control_point = ros::Time::now();
        last_time_cmd_vel = last_time_control_point;
        pid_initialized = false;

        ROS_INFO_STREAM("Got new global plan with " << plan.size() << " points.");


        return true;
    }

    bool FTCPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        calculateControlPoint();

        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(0.5));
        Eigen::Affine3d local_control_point;

        tf2::doTransform(current_control_point, local_control_point, map_to_base);


        double distance = local_control_point.translation().norm();


        if (distance > config.max_follow_dist) {
            ROS_ERROR_STREAM("Error: Robot was too far away. Stopping controller!");

            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            return false;
        } else if(distance < config.min_follow_dist) {
            ROS_INFO_STREAM("Point too close, waiting for point to move");


            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            return true;
        }


        ros::Time now = ros::Time::now();
        double dt = now.toSec() - last_time_cmd_vel.toSec();
        last_time_cmd_vel = now;





        double angle = atan2(local_control_point.translation().y(), local_control_point.translation().x());

        speed_factor = local_control_point.translation().normalized().x();
        if(speed_factor < 0.1) {
            speed_factor = 0.1;
        }


        double angle_d = (angle - last_angle_error) / dt;
        double speed_d = (distance - last_distance_error) / dt;


        last_angle_error = angle;
        last_distance_error = distance;







        // TODO: not really, we need to wait for bot and do a final rotation
        finished = current_index >= global_plan.size() - 1;

        if(finished) {
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
        } else {
            cmd_vel.angular.z = angle * config.kp_ang + angle_d * config.kd_ang;
            cmd_vel.linear.x = distance * config.kp_speed + speed_d * config.kd_speed;
        }

        return true;
    }


    bool FTCPlanner::isGoalReached() {
        return finished;
    }


    FTCPlanner::~FTCPlanner() {
        if (reconfig_server != nullptr) {
            delete reconfig_server;
            reconfig_server = nullptr;
        }
    }

    void FTCPlanner::calculateControlPoint() {
        ros::Time now = ros::Time::now();
        double dt = now.toSec() - last_time_control_point.toSec();
//        double allowed_speed = std::max(0.0, config.speed * (1.0 - 2.0 * (std::abs(last_angle) / M_PI)));
double allowed_speed = config.speed*speed_factor;
        double distance = dt * allowed_speed;

        ROS_INFO_STREAM("Speed: " << allowed_speed);

        Eigen::Affine3d nextPose, currentPose;
        while (distance > 0 && current_index < global_plan.size() - 1) {

            tf2::fromMsg(global_plan[current_index].pose, currentPose);
            tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);

            double pose_distance = (nextPose.translation() - currentPose.translation()).norm();
            if (pose_distance <= 0.0) {
                ROS_WARN_STREAM("Skipping duplicate point in global plan.");
                current_index++;
                continue;
            }
            double remaining_distance_to_next_pose = pose_distance * (1.0 - current_progress);

            ROS_INFO_STREAM("Distance to next: " << remaining_distance_to_next_pose);

            if (remaining_distance_to_next_pose < distance) {
                // we need to move further than the remaining distance. Skip to the next point and decrease distance.
                current_progress = 0.0;
                current_index++;
                distance -= remaining_distance_to_next_pose;
            } else {
                // we cannot reach the next point yet, so we update the percentage
                current_progress = (pose_distance * current_progress + distance) / pose_distance;
                if (current_progress > 1.0) {
                    ROS_WARN_STREAM("Progress > 1.0");
//                    current_progress = 1.0;
                }
                distance = 0;
            }
        }

        ROS_INFO_STREAM("Point: " << current_index << " progress: " << current_progress);

        if (current_index == global_plan.size() - 1) {
            ROS_INFO_STREAM("Reached last point!");
            tf2::fromMsg(global_plan[current_index].pose, current_control_point);
        } else {
            tf2::fromMsg(global_plan[current_index].pose, currentPose);
            tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);
            // interpolate between points
            Eigen::Quaternion<double> rot1(currentPose.linear());
            Eigen::Quaternion<double> rot2(nextPose.linear());

            Eigen::Vector3d trans1 = currentPose.translation();
            Eigen::Vector3d trans2 = nextPose.translation();

            Eigen::Affine3d result;
            result.translation() = (1.0 - current_progress) * trans1 + current_progress * trans2;
            result.linear() = rot1.slerp(current_progress, rot2).toRotationMatrix();

            current_control_point = result;
        }
        {
            geometry_msgs::PoseStamped viz;
            viz.header = global_plan[current_index].header;
            viz.pose = tf2::toMsg(current_control_point);
            global_point_pub.publish(viz);
        }

        last_time_control_point = now;
    }


}
