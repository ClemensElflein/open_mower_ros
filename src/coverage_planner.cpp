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
#include <PerimeterGenerator.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"


bool visualize_plan;
ros::Publisher marker_array_publisher;



void createLineMarkers(std::vector<Polygons> outline_groups,std::vector<Polygons> obstacle_groups, Polylines &fill_lines, visualization_msgs::MarkerArray &markerArray) {

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

    for(auto &group: outline_groups) {
        for (auto &line: group) {
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
                for (auto &pt: line.points) {
                    geometry_msgs::Point vpt;
                    vpt.x = unscale(pt.x);
                    vpt.y = unscale(pt.y);
                    marker.points.push_back(vpt);
                }

                markerArray.markers.push_back(marker);
            }
        }
        cidx = (cidx + 1) % colors.size();
    }

    for (auto &line: fill_lines) {
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
            for (auto &pt: line.points) {
                geometry_msgs::Point vpt;
                vpt.x = unscale(pt.x);
                vpt.y = unscale(pt.y);
                marker.points.push_back(vpt);
            }

            markerArray.markers.push_back(marker);

            cidx = (cidx + 1) % colors.size();
        }
    }
        for(auto &group: obstacle_groups) {
            for (auto &line: group) {
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
                    for (auto &pt: line.points) {
                        geometry_msgs::Point vpt;
                        vpt.x = unscale(pt.x);
                        vpt.y = unscale(pt.y);
                        marker.points.push_back(vpt);
                    }

                    markerArray.markers.push_back(marker);

                }
            }
            cidx = (cidx + 1) % colors.size();

        }
}

void traverse(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: contours) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

bool planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res) {

    Slic3r::Polygon outline_poly;
    for (auto &pt: req.outline.points) {
        outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
    }

    outline_poly.make_counter_clockwise();

    // This ExPolygon contains our input area with holes.
    Slic3r::ExPolygon expoly(outline_poly);

    for (auto &hole: req.holes) {
        Slic3r::Polygon hole_poly;
        for (auto &pt: hole.points) {
            hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }
        hole_poly.make_clockwise();

        expoly.holes.push_back(hole_poly);
    }





    // Results are stored here
    std::vector<Polygons> area_outlines;
    Polylines fill_lines;
    std::vector<Polygons> obstacle_outlines;




    coord_t distance = scale_(req.distance);
    coord_t outer_distance = scale_(req.outer_offset);

    // detect how many perimeters must be generated for this island
    int loops = req.outline_count;

    ROS_INFO_STREAM("generating " << loops << " outlines");

    const int loop_number = loops - 1;  // 0-indexed loops


    Polygons gaps;

    Polygons last = expoly;
    if (loop_number >= 0) {  // no loops = -1

        std::vector<PerimeterGeneratorLoops> contours(loop_number + 1);    // depth => loops
        std::vector<PerimeterGeneratorLoops> holes(loop_number + 1);       // depth => loops

        for (int i = 0; i <= loop_number; ++i) {  // outer loop is 0
            Polygons offsets;

            if (i == 0) {
                offsets = offset(
                        last,
                        -outer_distance
                );
            } else {
                offsets = offset(
                        last,
                        -distance
                );
            }

            if (offsets.empty()) break;


            last = offsets;

            for (Polygons::const_iterator polygon = offsets.begin(); polygon != offsets.end(); ++polygon) {
                PerimeterGeneratorLoop loop(*polygon, i);
                loop.is_contour = polygon->is_counter_clockwise();
                if (loop.is_contour) {
                    contours[i].push_back(loop);
                } else {
                    holes[i].push_back(loop);
                }
            }
        }

        // nest loops: holes first
        for (int d = 0; d <= loop_number; ++d) {
            PerimeterGeneratorLoops &holes_d = holes[d];

            // loop through all holes having depth == d
            for (int i = 0; i < (int) holes_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = holes_d[i];

                // find the hole loop that contains this one, if any
                for (int t = d + 1; t <= loop_number; ++t) {
                    for (int j = 0; j < (int) holes[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = holes[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            holes_d.erase(holes_d.begin() + i);
                            --i;
                            goto NEXT_LOOP;
                        }
                    }
                }

                NEXT_LOOP:;
            }
        }

        // nest contour loops
        for (int d = loop_number; d >= 1; --d) {
            PerimeterGeneratorLoops &contours_d = contours[d];

            // loop through all contours having depth == d
            for (int i = 0; i < (int) contours_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = contours_d[i];

                // find the contour loop that contains it
                for (int t = d - 1; t >= 0; --t) {
                    for (size_t j = 0; j < contours[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = contours[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            contours_d.erase(contours_d.begin() + i);
                            --i;
                            goto NEXT_CONTOUR;
                        }
                    }
                }

                NEXT_CONTOUR:;
            }
        }

        traverse(contours[0], area_outlines);
        for(auto &hole:holes) {
            traverse(hole, obstacle_outlines);
        }

        for(auto &obstacle_group : obstacle_outlines) {
            std::reverse(obstacle_group.begin(), obstacle_group.end());
        }

    }




    ExPolygons expp = union_ex(last);


    // Go through the innermost poly and create the fill path using a Fill object
    for (auto &poly: expp) {
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
        append_to(fill_lines, lines);
        delete fill;
        fill = nullptr;

        ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
        for (int i = 0; i < lines.size(); i++) {
            ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
        }
    }



    if (visualize_plan) {


        visualization_msgs::MarkerArray arr;
        createLineMarkers(area_outlines, obstacle_outlines, fill_lines, arr);
        marker_array_publisher.publish(arr);
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;

    for(auto &group:area_outlines) {
        slic3r_coverage_planner::Path path;
        path.is_outline = true;
        path.path.header = header;
        int split_index = 0;
        for (int i = 0; i < group.size(); i++) {
            auto &poly = group[i];

            Polyline line;
            if(split_index < poly.points.size()) {
                line = poly.split_at_index(split_index);
            } else {
                line = poly.split_at_first_point();
                split_index = 0;
            }
            split_index+=2;
            line.remove_duplicate_points();



            auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
            if (equally_spaced_points.size() < 2) {
                ROS_INFO("Skipping single dot");
                continue;
            }
            ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

            Point *lastPoint = nullptr;
            for (auto &pt: equally_spaced_points) {
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
                path.path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);

        }
        res.paths.push_back(path);
    }

    for (int i = 0; i < fill_lines.size(); i++) {
        auto &line = fill_lines[i];
        slic3r_coverage_planner::Path path;
        path.is_outline = false;
        path.path.header = header;

        line.remove_duplicate_points();


        auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
        if (equally_spaced_points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

        Point *lastPoint = nullptr;
        for (auto &pt: equally_spaced_points) {
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
            path.path.poses.push_back(pose);
            lastPoint = &pt;
        }

        // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose.orientation = path.path.poses.back().pose.orientation;
        pose.pose.position.x = unscale(lastPoint->x);
        pose.pose.position.y = unscale(lastPoint->y);
        pose.pose.position.z = 0;
        path.path.poses.push_back(pose);

        res.paths.push_back(path);
    }

    for(auto &group:obstacle_outlines) {
        slic3r_coverage_planner::Path path;
        path.is_outline = true;
        path.path.header = header;
        int split_index = 0;
        for (int i = 0; i < group.size(); i++) {
            auto &poly = group[i];

            Polyline line;
            if(split_index < poly.points.size()) {
                line = poly.split_at_index(split_index);
            } else {
                line = poly.split_at_first_point();
                split_index = 0;
            }
            split_index+=2;
            line.remove_duplicate_points();



            auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
            if (equally_spaced_points.size() < 2) {
                ROS_INFO("Skipping single dot");
                continue;
            }
            ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

            Point *lastPoint = nullptr;
            for (auto &pt: equally_spaced_points) {
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
                path.path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last poe
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);

        }
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
