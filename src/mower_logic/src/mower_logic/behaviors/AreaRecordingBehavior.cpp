// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "AreaRecordingBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern ros::ServiceClient emergencyClient;
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern mower_msgs::Status last_status;
extern ros::NodeHandle *n;

extern void stop();

extern bool setGPS(bool enabled);

AreaRecordingBehavior AreaRecordingBehavior::INSTANCE;


std::string AreaRecordingBehavior::state_name() {
    return "AREA_RECORDING";
}

Behavior *AreaRecordingBehavior::execute() {
    bool error = false;
    ros::Rate inputDelay(ros::Duration().fromSec(0.1));
    while(ros::ok() && !aborted) {
        mower_map::MapArea result;
        bool has_outline = false;



        while (ros::ok() && !finished_all && !error && !aborted) {

            if(set_docking_position) {
                geometry_msgs::Pose pos;
                if(getDockingPosition(pos)) {

                    ROS_INFO_STREAM("new docking pos = " << pos);

                    mower_map::SetDockingPointSrv set_docking_point_srv;
                    set_docking_point_srv.request.docking_pose = pos;
                    auto result = set_docking_point_client.call(set_docking_point_srv);

                    has_first_docking_pos = false;
                }

                set_docking_position = false;
            }

            if (poly_recording_enabled) {
                geometry_msgs::Polygon poly;
                bool success = recordNewPolygon(poly);
                if (success) {
                    if (!has_outline) {
                        // first polygon is outline
                        has_outline = true;
                        result.area = poly;

                        std_msgs::ColorRGBA color;
                        color.r = 0.0f;
                        color.g = 1.0f;
                        color.b = 0.0f;
                        color.a = 1.0f;

                        marker.color = color;
                        marker.id = markers.markers.size() + 1;
                        marker.action = visualization_msgs::Marker::ADD;
                        markers.markers.push_back(marker);
                    } else {
                        // we already have an outline, add obstacles
                        result.obstacles.push_back(poly);

                        std_msgs::ColorRGBA color;
                        color.r = 1.0f;
                        color.g = 0.0f;
                        color.b = 0.0f;
                        color.a = 1.0f;

                        marker.color = color;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.id = markers.markers.size() + 1;
                        markers.markers.push_back(marker);

                    }

                } else {
                    error = true;
                    ROS_ERROR_STREAM("Error during poly record");
                }
                marker_array_pub.publish(markers);
            }

            inputDelay.sleep();
        }


        if(!error && has_outline) {
            if(is_mowing_area) {
                ROS_INFO_STREAM("Area recording completed. Adding mowing area.");
            } else {
                ROS_INFO_STREAM("Area recording completed. Adding navigation area.");
            }
            mower_map::AddMowingAreaSrv srv;
            srv.request.isNavigationArea = !is_mowing_area;
            srv.request.area = result;
            if (add_mowing_area_client.call(srv)) {
                ROS_INFO_STREAM("Area added successfully");
            } else {
                ROS_ERROR_STREAM("error adding area");
            }
        }

        // reset recording error for next area
        error = false;
        // reset finished all in case we want to record a second area
        finished_all = false;
    }

    return &IdleBehavior::INSTANCE;

}

void AreaRecordingBehavior::enter() {
    has_first_docking_pos = false;
    has_odom = false;
    poly_recording_enabled = false;
    finished_all = false;
    set_docking_position = false;
    markers = visualization_msgs::MarkerArray();



    add_mowing_area_client = n->serviceClient<mower_map::AddMowingAreaSrv>("mower_map_service/add_mowing_area");
    set_docking_point_client = n->serviceClient<mower_map::SetDockingPointSrv>("mower_map_service/set_docking_point");

    marker_pub = n->advertise<visualization_msgs::Marker>("area_recorder/progress_visualization", 10);
    marker_array_pub = n->advertise<visualization_msgs::MarkerArray>("area_recorder/progress_visualization_array", 10);


    ROS_INFO_STREAM("Starting recording area");

    ROS_INFO_STREAM("Subscribing to /joy for user input");
    joy_sub = n->subscribe("/joy", 100,
                                          &AreaRecordingBehavior::joy_received, this);
    odom_sub = n->subscribe("mower/odom", 100,
                                           &AreaRecordingBehavior::odom_received, this);

}

void AreaRecordingBehavior::exit() {
    marker_pub.shutdown();
    marker_array_pub.shutdown();
    joy_sub.shutdown();
    odom_sub.shutdown();
    add_mowing_area_client.shutdown();
    set_docking_point_client.shutdown();
}

void AreaRecordingBehavior::reset() {
}

bool AreaRecordingBehavior::needs_gps() {
    // we only need GPS if we're in approach mode
    return false;
}

bool AreaRecordingBehavior::mower_enabled() {
    // No mower during docking
    return false;
}

void AreaRecordingBehavior::odom_received(const nav_msgs::Odometry &odom_msg) {
    last_odom = odom_msg;
    has_odom = true;
}
void AreaRecordingBehavior::joy_received(const sensor_msgs::Joy &joy_msg) {

    if (joy_msg.buttons[1] && !last_joy.buttons[1]) {
        // B was pressed. We toggle recording state
        ROS_INFO_STREAM("B PRESSED");
        poly_recording_enabled = !poly_recording_enabled;
    }
    // Y + up was pressed, we finish the recording for a navigation area
    if ((joy_msg.buttons[3] && joy_msg.axes[7] > 0.5) && !(last_joy.buttons[3] && last_joy.axes[7] > 0.5)) {
        ROS_INFO_STREAM("Y + UP PRESSED, recording navigation area");
        // stop current poly recording
        poly_recording_enabled = false;

        // set finished
        is_mowing_area = false;
        finished_all = true;
    }
    // Y + down was pressed, we finish the recording for a navigation area
    if ((joy_msg.buttons[3] && joy_msg.axes[7] < -0.5) && !(last_joy.buttons[3] && last_joy.axes[7] < -0.5)) {
        ROS_INFO_STREAM("Y + DOWN PRESSED, recording mowing area");
        // stop current poly recording
        poly_recording_enabled = false;

        // set finished
        is_mowing_area = true;
        finished_all = true;
    }

    // X was pressed, set base position if we are not currently recording
    if (joy_msg.buttons[2] && !last_joy.buttons[2]) {
        ROS_INFO_STREAM("X PRESSED");
        set_docking_position = true;
    }



    last_joy = joy_msg;
}


bool AreaRecordingBehavior::recordNewPolygon(geometry_msgs::Polygon &polygon) {

    ROS_INFO_STREAM("recordNewPolygon");


    bool success = true;
    marker = visualization_msgs::Marker();
    marker.header.frame_id = "map";
    marker.ns = "area_recorder";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = 0;
    marker.pose.orientation.w = 1.0f;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.frame_locked = true;

    std_msgs::ColorRGBA color;
    color.b = 1.0f;
    color.a = 1.0f;

    marker.color = color;

    ros::Rate updateRate(10);

    has_odom = false;

    while (true) {
        if (!ros::ok() || aborted) {
            ROS_WARN_STREAM("Preempting Area Recorder");
            success = false;
            break;
        }

        updateRate.sleep();

        if(!has_odom)
            continue;

        auto pose_in_map = last_odom.pose.pose;
        if (polygon.points.empty()) {
            // add the first point
            geometry_msgs::Point32 pt;
            pt.x = pose_in_map.position.x;
            pt.y = pose_in_map.position.y;
            pt.z = 0.0;
//                ROS_INFO_STREAM("Adding First Point: " << pt);

            polygon.points.push_back(pt);
            {
                geometry_msgs::Point vpt;
                vpt.x = pt.x;
                vpt.y = pt.y;
                marker.points.push_back(vpt);
            }

            marker.header.seq++;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "map";

            marker_pub.publish(marker);

            polygon.points.push_back(pt);
        } else {

            auto last = polygon.points.back();
            tf2::Vector3 last_point(last.x, last.y, 0.0);
            tf2::Vector3 current_point(pose_in_map.position.x, pose_in_map.position.y, 0.0);

            if ((current_point - last_point).length() > 0.1) {
                geometry_msgs::Point32 pt;
                pt.x = pose_in_map.position.x;
                pt.y = pose_in_map.position.y;
                pt.z = 0.0;
//                    ROS_INFO_STREAM("Adding Point: " << pt);
                polygon.points.push_back(pt);
                {
                    geometry_msgs::Point vpt;
                    vpt.x = pt.x;
                    vpt.y = pt.y;
                    marker.points.push_back(vpt);
                }

                marker.header.seq++;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = "map";

                marker_pub.publish(marker);
            }
        }

        if (!poly_recording_enabled) {
            if(polygon.points.size() > 2) {
                // add first point to close the poly
                polygon.points.push_back(polygon.points.front());
            } else {
                success = false;
            }
            ROS_INFO_STREAM("Finished Recording polygon");
            break;
        }
    }

    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);



    return success;
}

bool AreaRecordingBehavior::getDockingPosition(geometry_msgs::Pose &pos) {
    if(!has_first_docking_pos) {
        ROS_INFO_STREAM("Recording first docking position");
        auto odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("mower/odom", ros::Duration(1, 0));

        first_docking_pos = odom_ptr->pose.pose;
        has_first_docking_pos = true;
        return false;
    } else {
        ROS_INFO_STREAM("Recording second docking position");
        auto odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("mower/odom", ros::Duration(1, 0));

        pos.position = odom_ptr->pose.pose.position;

        double yaw = atan2(pos.position.y - first_docking_pos.position.y,pos.position.x -  first_docking_pos.position.x);
        tf2::Quaternion docking_orientation(0.0, 0.0, yaw);
        pos.orientation = tf2::toMsg(docking_orientation);

        return true;
    }
}

void AreaRecordingBehavior::command_home() {
    abort();
}

void AreaRecordingBehavior::command_start() {

}

void AreaRecordingBehavior::command_s1() {

}

void AreaRecordingBehavior::command_s2() {

}

bool AreaRecordingBehavior::redirect_joystick() {
    return true;
}
