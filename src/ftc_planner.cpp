
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
        global_plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1, true);

        costmap = costmap_ros;
        tf_buffer = tf;

        //Parameter for dynamic reconfigure
        reconfig_server = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this,
                                                                                     _1, _2);
        reconfig_server->setCallback(cb);

        planner_step = 0;
        goal_reached = false;


        ROS_INFO("FTCPlanner: Version 2 Init.");
    }

    void FTCPlanner::reconfigureCB(FTCPlannerConfig &c, uint32_t level) {
        if (c.restore_defaults) {
            reconfig_server->getConfigDefault(c);
            c.restore_defaults = false;
        }
        config = c;

        // just to be sure
        current_movement_speed = config.speed_slow;
    }

    bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
        planner_step = 0;
        goal_reached = false;
        global_plan = plan;
        current_index = 0;
        current_progress = 0.0;
        last_time = ros::Time::now();
        current_movement_speed = config.speed_slow;

        if(global_plan.size() > 2) {
            // duplicate last point
            global_plan.push_back(global_plan.back());
            // give second from last point last oriantation as the point before that
            global_plan[global_plan.size()-2].pose.orientation = global_plan[global_plan.size()-3].pose.orientation;
        }

        nav_msgs::Path path;
        if(plan.size() > 0) {
            path.header = plan.front().header;
            path.poses = plan;
        }
        global_plan_pub.publish(path);

        ROS_INFO_STREAM("Got new global plan with " << plan.size() << " points.");


        return true;
    }

    bool FTCPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        ros::Time now = ros::Time::now();
        dt = now.toSec() - last_time.toSec();
        last_time = now;

        bool success = computeVelocityCommandsFollow(cmd_vel);


        // allow planner to finish in the last two phases (position, orientation)
        if(planner_step == 2) {
            double time_since_finished = (ros::Time::now() - planner_rotation_time).toSec();
            if(time_since_finished > config.goal_timeout) {
                ROS_WARN_STREAM("Setting goal finished due to timeout");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;

                goal_reached = true;
                return success;
            }

            // wait for robot to get to the goal pose
            auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(0.5));
            Eigen::Affine3d local_control_point;
            tf2::doTransform(current_control_point, local_control_point, map_to_base);


            double goal_distance = local_control_point.translation().norm();
            double angle_error = local_control_point.rotation().eulerAngles(0,1,2).z();

            // Not yet finished
            if(planner_step < 2 || goal_distance > config.max_goal_distance_error) {
                return success;
            }

            // Not yet finished
            if(planner_step < 2 || abs(angle_error) > config.max_goal_angle_error * (M_PI/180.0)) {
                return success;
            }

            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;

            goal_reached = true;
        }

        return success;
    }


    bool FTCPlanner::isGoalReached() {
        // Since we want to ensure that the last command was 0 speeds, we need to calculate finished state in computeVelocityCommands
        return goal_reached;
    }


    FTCPlanner::~FTCPlanner() {
        if (reconfig_server != nullptr) {
            delete reconfig_server;
            reconfig_server = nullptr;
        }
    }

    int FTCPlanner::moveControlPoint() {
        if(planner_step == 0) {
            // Normal planner operation
            double straight_dist = distanceLookahead();
            double speed;
            if(straight_dist >= config.speed_fast_threshold) {
                speed = config.speed_fast;
            } else {
                speed = config.speed_slow;
            }

            if(speed > current_movement_speed) {
                // accelerate
                current_movement_speed += dt*config.acceleration;
                if(current_movement_speed > speed)
                    current_movement_speed = speed;
            } else if(speed < current_movement_speed) {
                // decelerate
                current_movement_speed -= dt*config.acceleration;
                if(current_movement_speed < speed)
                    current_movement_speed = speed;
            }

            double distance_to_move = dt * current_movement_speed;

            Eigen::Affine3d nextPose, currentPose;
            while (distance_to_move > 0 && current_index < global_plan.size() - 2) {

                tf2::fromMsg(global_plan[current_index].pose, currentPose);
                tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);

                double pose_distance = (nextPose.translation() - currentPose.translation()).norm();
                if (pose_distance <= 0.0) {
                    ROS_WARN_STREAM("Skipping duplicate point in global plan.");
                    current_index++;
                    continue;
                }
                double remaining_distance_to_next_pose = pose_distance * (1.0 - current_progress);

//                ROS_INFO_STREAM("Distance to next: " << remaining_distance_to_next_pose);

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

//            ROS_INFO_STREAM("Point: " << current_index << " progress: " << current_progress);

            if (current_index == global_plan.size() - 2) {
                ROS_INFO_STREAM("switching planner to position mode");
                tf2::fromMsg(global_plan[current_index].pose, current_control_point);
                {
                    geometry_msgs::PoseStamped viz;
                    viz.header = global_plan[current_index].header;
                    viz.pose = tf2::toMsg(current_control_point);
                    global_point_pub.publish(viz);
                }
                // wait for robot to approach last point
                return 1;
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

            // stay in pahse 0
            return 0;
        } else if(planner_step == 1) {
            // we are waiting for the robot to approach the final point before switching to the real last point (only difference here is orientation of the two)
            auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(0.5));
            Eigen::Affine3d local_control_point;
            tf2::doTransform(current_control_point, local_control_point, map_to_base);

            double position_time = (ros::Time::now() - planner_position_time).toSec();

            double goal_distance = local_control_point.translation().norm();
            if(goal_distance < config.max_goal_distance_error || position_time > config.goal_timeout) {
                // we can now switch to the last step
                ROS_INFO_STREAM("switching planner to rotation mode");
                current_index++;
                tf2::fromMsg(global_plan[current_index].pose, current_control_point);
                {
                    geometry_msgs::PoseStamped viz;
                    viz.header = global_plan[current_index].header;
                    viz.pose = tf2::toMsg(current_control_point);
                    global_point_pub.publish(viz);
                }
                return 2;
            }
        }

        return planner_step;
    }

    bool FTCPlanner::computeVelocityCommandsFollow(geometry_msgs::Twist &cmd_vel) {
        // check, if we're completely done
        if(goal_reached)
            return true;

        int planner_step_new = moveControlPoint();

        // start timeout if phase switched to 1
        if(planner_step == 0 && planner_step_new == 1) {
            planner_position_time = ros::Time::now();
        } else if(planner_step == 1 && planner_step_new == 2) {
            planner_rotation_time = ros::Time::now();
        }
        planner_step = planner_step_new;


        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(0.5));
        Eigen::Affine3d local_control_point;
        tf2::doTransform(current_control_point, local_control_point, map_to_base);


        double distance = local_control_point.translation().norm();

        // check for crash first
        if(distance > config.max_follow_distance) {
            ROS_ERROR_STREAM("Robot is far away from global plan. It probably has crashed.");
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            return false;
        }

        double lat_error = local_control_point.translation().y();
        double lon_error = local_control_point.translation().x();
        double angle_error = local_control_point.rotation().eulerAngles(0,1,2).z();

        i_lon_error += lon_error * dt;
        i_lat_error += lat_error * dt;
        i_angle_error += angle_error * dt;

        if(i_lon_error > config.ki_lon_max) {
            i_lon_error = config.ki_lon_max;
        } else if(i_lon_error < -config.ki_lon_max) {
            i_lon_error = -config.ki_lon_max;
        }
        if(i_lat_error > config.ki_lat_max) {
            i_lat_error = config.ki_lat_max;
        } else if(i_lat_error < -config.ki_lat_max) {
            i_lat_error = -config.ki_lat_max;
        }
        if(i_angle_error > config.ki_ang_max) {
            i_angle_error = config.ki_ang_max;
        } else if(i_angle_error < -config.ki_ang_max) {
            i_angle_error = -config.ki_ang_max;
        }


        double d_lat = (lat_error - last_lat_error) / dt;
        double d_lon = (lon_error - last_lon_error) / dt;
        double d_angle = (angle_error - last_angle_error) / dt;




        last_lat_error = lat_error;
        last_lon_error = lon_error;
        last_angle_error = angle_error;



        double ang_speed = angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang +
                lat_error * config.kp_lat + i_lat_error * config.ki_lat + d_lat * config.kd_lat;
        if (ang_speed > config.max_cmd_vel_ang) {
            ang_speed = config.max_cmd_vel_ang;
        } else if (ang_speed < -config.max_cmd_vel_ang) {
            ang_speed = -config.max_cmd_vel_ang;
        }

        cmd_vel.angular.z = ang_speed;

        double lin_speed = lon_error * config.kp_lon + i_lon_error * config.ki_lon + d_lon * config.kd_lon;
        if (lin_speed < 0) {
            lin_speed = 0;
        } else if (lin_speed > config.max_cmd_vel_speed) {
            lin_speed = config.max_cmd_vel_speed;
        }
        cmd_vel.linear.x = lin_speed;


        return true;
    }

    double FTCPlanner::distanceLookahead() {
        if(global_plan.size() < 2) {
            return 0;
        }
        Eigen::Quaternion<double> current_rot(current_control_point.linear());

        Eigen::Affine3d last_straight_point = current_control_point;
        for(uint32_t i = current_index+1; i < global_plan.size(); i++) {
            tf2::fromMsg(global_plan[i].pose, last_straight_point);
            // check, if direction is the same. if so, we add the distance
            Eigen::Quaternion<double> rot2(last_straight_point.linear());
            if(abs(rot2.angularDistance(current_rot)) > 5.0 * (M_PI/180.0)) {
                break;
            }
        }

        return (last_straight_point.translation() - current_control_point.translation()).norm();
    }


}
