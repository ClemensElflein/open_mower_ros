
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
        last_time = ros::Time::now();

        rotate_mode = false;

        ROS_INFO_STREAM("Got new global plan with " << plan.size() << " points.");


        return true;
    }

    bool FTCPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        ros::Time now = ros::Time::now();
        dt = now.toSec() - last_time.toSec();
        last_time = now;
        if (rotate_mode) {
            return computeVelocityCommandsRotate(cmd_vel);
        }
        return computeVelocityCommandsFollow(cmd_vel);

    }


    bool FTCPlanner::isGoalReached() {
        return finished && !rotate_mode;
    }


    FTCPlanner::~FTCPlanner() {
        if (reconfig_server != nullptr) {
            delete reconfig_server;
            reconfig_server = nullptr;
        }
    }

    void FTCPlanner::moveControlPoint() {
        double distance_to_move = dt * config.speed;

        Eigen::Affine3d nextPose, currentPose;
        while (distance_to_move > 0 && current_index < global_plan.size() - 1) {

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

            if (remaining_distance_to_next_pose < distance_to_move) {
                // we need to move further than the remaining distance_to_move. Skip to the next point and decrease distance_to_move.
                current_progress = 0.0;
                current_index++;
                distance_to_move -= remaining_distance_to_next_pose;
            } else {
                // we cannot reach the next point yet, so we update the percentage
                current_progress = (pose_distance * current_progress + distance_to_move) / pose_distance;
                if (current_progress > 1.0) {
                    ROS_WARN_STREAM("Progress > 1.0");
//                    current_progress = 1.0;
                }
                distance_to_move = 0;
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

    }

    bool FTCPlanner::computeVelocityCommandsFollow(geometry_msgs::Twist &cmd_vel) {

        if (finished) {
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            return true;
        }

        moveControlPoint();


        // TODO: not really, we need to wait for bot and do a final rotation
        bool new_finished = current_index >= global_plan.size() - 1;

        // enable rotation at the end
        if (new_finished) {
            rotate_mode = true;

            tf2::Quaternion quat;
            tf2::fromMsg(global_plan[current_index].pose.orientation, quat);
            tf2::Matrix3x3 m(quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            rotation_target_angle = fmod(yaw, M_PI*2.0);
            if(rotation_target_angle < 0) {
                rotation_target_angle+= M_PI*2.0;
            }
        }
        finished = new_finished;

        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(0.5));
        Eigen::Affine3d local_control_point;
        tf2::doTransform(current_control_point, local_control_point, map_to_base);


        double distance = local_control_point.translation().norm();
        double angle = atan2(local_control_point.translation().y(), local_control_point.translation().x());


        if (distance > config.max_follow_dist) {
            ROS_ERROR_STREAM("Error: Robot was too far away. Stopping controller!");

            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            return false;
        } else if (distance < config.min_follow_dist) {
            ROS_INFO_STREAM("Point too close, waiting for point to move");


            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            return true;
        }


        if (abs(angle) > config.max_angle_follow_mode * (M_PI / 180.0)) {
            ROS_INFO_STREAM("Switching to rotation mode. Angle was: " << (angle * (180.0/M_PI)));

            tf2::Quaternion quat;
            tf2::fromMsg(map_to_base.transform.rotation, quat);

            tf2::Matrix3x3 m(quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // rotation target angle between -M_PI and M_PI
            rotation_target_angle = fmod(angle - yaw, M_PI*2.0);
            if(rotation_target_angle < 0) {
                rotation_target_angle+= M_PI*2.0;
            }
            rotate_mode = true;
            return computeVelocityCommandsRotate(cmd_vel);
        }


        double angle_d = (angle - last_angle_error) / dt;
        double speed_d = (distance - last_distance_error) / dt;


        last_angle_error = angle;
        last_distance_error = distance;


        double ang_speed = angle * config.kp_ang + angle_d * config.kd_ang;
        if (ang_speed > config.max_cmd_vel_ang) {
            ang_speed = config.max_cmd_vel_ang;
        } else if (ang_speed < -config.max_cmd_vel_ang) {
            ang_speed = -config.max_cmd_vel_ang;
        }
        cmd_vel.angular.z = ang_speed;
        double lin_speed = distance * config.kp_speed + speed_d * config.kd_speed;
        if (lin_speed < 0) {
            lin_speed = 0;
        } else if (lin_speed > config.max_cmd_vel_speed) {
            lin_speed = config.max_cmd_vel_speed;
        }
        cmd_vel.linear.x = lin_speed;


        return true;
    }

    bool FTCPlanner::computeVelocityCommandsRotate(geometry_msgs::Twist &cmd_vel) {
        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(0.5));

        tf2::Quaternion quat;
        tf2::fromMsg(map_to_base.transform.rotation, quat);

        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw *= -1.0;

        double angle = fmod( (rotation_target_angle - yaw) + M_PI, M_PI*2.0);
        if(angle < 0)
            angle += 2.0*M_PI;
        angle -= M_PI;

        ROS_INFO_STREAM("angle: " << (rotation_target_angle* (180.0 / M_PI)) << ", current: " << (yaw* (180.0 / M_PI)) << ", angle error: "
                                  << (angle * (180.0 / M_PI)));



        double angle_d = (angle - last_angle_error) / dt;


        last_angle_error = angle;


        double ang_speed = angle * config.kp_ang + angle_d * config.kd_ang;
        if (ang_speed > config.max_cmd_vel_ang) {
            ang_speed = config.max_cmd_vel_ang;
        } else if (ang_speed < -config.max_cmd_vel_ang) {
            ang_speed = -config.max_cmd_vel_ang;
        }
        cmd_vel.angular.z = ang_speed;
        cmd_vel.linear.x = 0;

        if (abs(angle) < config.min_angle_rotation_mode * (M_PI / 180.0)) {
            ROS_INFO_STREAM("Switching to follow mode");
            rotate_mode = false;
        }

        return true;
    }


}
