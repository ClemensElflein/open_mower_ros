//
// Created by Clemens Elflein on 27.08.21.
//

#include "ros/ros.h"

#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Fill/FillConcentric.hpp"


#include "slic3r_coverage_planner/PlanPath.h"
#include "visualization_msgs/MarkerArray.h"
#include "Surface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <Fill/FillPlanePath.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ClipperUtils.hpp"


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

            cidx = (cidx + 1) % colors.size();
        }
    }

}


bool planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res) {

    Slic3r::Polygon outline_poly;
    for (auto &pt : req.outline.points) {
        outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
    }

    outline_poly.make_clockwise();

    // This ExPolygon contains our input area with holes.
    Slic3r::ExPolygon expoly(outline_poly);

    for (auto &hole : req.holes) {
        Slic3r::Polygon hole_poly;
        for (auto &pt : hole.points) {
            hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }
        hole_poly.make_counter_clockwise();

        expoly.holes.push_back(hole_poly);
    }



    // Results are stored here
    Polylines all_lines;

    // We want to trace the outline first, so we build it
    Polyline outline(expoly.contour);

    // We want to trace a second time with a little offset.
    Slic3r::ExPolygons polys = offset_ex(expoly, -scale_(req.distance / 2));

    if(polys.size() == 1) {
        // we have a single second outline, we can chain them together for a nice flow
        Polygon c = polys.front().contour;
        c.make_clockwise();

        // align start with last end
        auto last_point = outline.last_point();
        Polyline new_line = Polyline(c);

        if(!new_line.points.empty()) {
            // find closest point
            double min_dist = FLT_MAX;
            Point closest_point;
            for(auto &pt : new_line.points) {
                double dist = pt.distance_to(last_point);
                if(dist < min_dist) {
                    closest_point = pt;
                    min_dist = dist;
                }
            }

            Polyline before,after;
            new_line.split_at(closest_point, &before, &after);

            outline.append(after);
            outline.append(before);
        }
        all_lines.push_back(outline);
    } else {
        // push the outline as is
        all_lines.push_back(outline);

        // add other poly outlines as well
        for (auto &poly : polys) {
            Polygon c = poly.contour;
            c.make_clockwise();
            all_lines.push_back(Polyline(c));
        }
    }





    // Go through the innermost poly and create the fill path using a Fill object
    for (auto &poly : polys) {
        Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, poly);


        Slic3r::Fill *fill;
        if (req.fill_type == slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR) {
            fill = new Slic3r::FillRectilinear();
        } else {
            fill = new Slic3r::FillConcentric();
        }
        fill->link_max_length = scale_(1.0);
        fill->angle = req.angle;
        fill->z = scale_(1.0);
        fill->endpoints_overlap = 0;
        fill->density = 1.0;
        fill->dont_connect = false;
        fill->dont_adjust = false;
        fill->min_spacing = req.distance;
        fill->complete = false;
        fill->link_max_length = 0;

        ROS_INFO_STREAM("Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

        Slic3r::Polylines lines = fill->fill_surface(surface);
        append_to(all_lines, lines);
        delete fill;
        fill = nullptr;

        ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
        for (int i = 0; i < lines.size(); i++) {
            ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
        }
    }

    for(auto &hole : expoly.holes) {
        all_lines.push_back(Polyline(hole));
    }
    for (auto &poly : polys) {
        for(auto &hole : poly.holes) {
            all_lines.push_back(Polyline(hole));
        }
    }


    

    if (visualize_plan) {
        visualization_msgs::MarkerArray arr;
        createLineMarkers(all_lines, arr);
        marker_array_publisher.publish(arr);
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;

    for (auto &line : all_lines) {

        nav_msgs::Path path;
        path.header = header;

        line.remove_duplicate_points();


        auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
        if (equally_spaced_points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

        Point *lastPoint = nullptr;
        for (auto &pt:equally_spaced_points) {
            if (lastPoint == nullptr) {
                lastPoint = &pt;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point

            auto dir = pt - *lastPoint;
            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.poses.push_back(pose);
            lastPoint = &pt;
        }

        // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.orientation = path.poses.back().pose.orientation;
        pose.pose.position.x = unscale(lastPoint->x);
        pose.pose.position.y = unscale(lastPoint->y);
        pose.pose.position.z = 0;
        path.poses.push_back(pose);

        res.paths.push_back(path);
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "slic3r_coverage_planner");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    visualize_plan = paramNh.param("visualize_plan", true);

    if (visualize_plan) {
        marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>(
                "slic3r_coverage_planner/path_marker_array", 100, true);
    }

    ros::ServiceServer plan_path_srv = n.advertiseService("slic3r_coverage_planner/plan_path", planPath);

    ros::spin();
    return 0;
}
