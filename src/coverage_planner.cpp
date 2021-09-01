//
// Created by Clemens Elflein on 27.08.21.
//

#include "ros/ros.h"

#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"

#include "slic3r_coverage_planner/PlanPath.h"
#include "visualization_msgs/MarkerArray.h"
#include "Surface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


bool visualize_plan;
ros::Publisher marker_array_publisher;



void createLineMarkers(Polylines &lines, visualization_msgs::MarkerArray &markerArray) {

    std::vector<std_msgs::ColorRGBA> colors;

    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }

    uint32_t cidx = 0;

    for (auto &line:lines) {
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.color = colors[cidx];
            marker.pose.orientation.w = 1;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
            for (auto &pt : line.points) {
                geometry_msgs::Point vpt;
                vpt.x = unscale(pt.x);
                vpt.y = unscale(pt.y);
                marker.points.push_back(vpt);
            }

            markerArray.markers.push_back(marker);

            cidx = (cidx+1) % colors.size();
        }
    }

}


bool planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res) {

    Slic3r::Polygon outline_poly;
    for(auto &pt : req.outline.points) {
        outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
    }

    outline_poly.make_clockwise();

    Slic3r::ExPolygon expoly(outline_poly);

    for(auto &hole : req.holes) {
        Slic3r::Polygon hole_poly;
        for(auto &pt : hole.points) {
            hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }
        hole_poly.make_counter_clockwise();

        expoly.holes.push_back(hole_poly);
    }

    Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, expoly);


    Slic3r::FillRectilinear fill;
    fill.link_max_length = scale_(1.0);
    fill.angle = req.angle;
    fill.z = scale_(1.0);
    fill.endpoints_overlap = 0;
    fill.density = 1.0;
    fill.dont_connect = false;
    fill.dont_adjust = false;
    fill.min_spacing = 0.15;
    fill.complete = false;
    fill.link_max_length = 0;

    ROS_INFO_STREAM("Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

    Slic3r::Polylines lines = fill.fill_surface(surface);

    ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
    for (int i = 0; i < lines.size(); i++) {
        ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
    }

    if(visualize_plan) {
        visualization_msgs::MarkerArray arr;
        createLineMarkers(lines, arr);
        marker_array_publisher.publish(arr);
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;
    res.path.header = header;
    for(auto &line : lines) {
        if(line.points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }

        Point *lastPoint = nullptr;
        for(auto &pt:line.points) {
            if(lastPoint == nullptr) {
                lastPoint = &pt;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point

            auto dir = pt - *lastPoint;
            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation= tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            res.path.poses.push_back(pose);
            lastPoint = &pt;
        }

        // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.orientation = res.path.poses.back().pose.orientation;
        pose.pose.position.x = unscale(lastPoint->x);
        pose.pose.position.y = unscale(lastPoint->y);
        pose.pose.position.z = 0;
        res.path.poses.push_back(pose);
    }


    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "slic3r_coverage_planner");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    visualize_plan = paramNh.param("visualize_plan", true);

    if(visualize_plan) {
        marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>("slic3r_coverage_planner/path_marker_array", 100, true);
    }

    ros::ServiceServer plan_path_srv = n.advertiseService("slic3r_coverage_planner/plan_path", planPath);

    ros::spin();
    return 0;
}
