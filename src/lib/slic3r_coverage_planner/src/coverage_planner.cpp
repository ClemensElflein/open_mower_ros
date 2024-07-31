//
// Created by Clemens Elflein on 27.08.21.
//

#include "ros/ros.h"

#include <boost/range/adaptor/reversed.hpp>

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


void
createMarkers(const slic3r_coverage_planner::PlanPathRequest &planning_request,
              const slic3r_coverage_planner::PlanPathResponse &planning_result,
              visualization_msgs::MarkerArray &markerArray) {

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

    // Create markers for the input polygon
    {
        // Walk through the paths we send to the navigation stack
        auto &path = planning_request.outline.points;
        // Each group gets a single line strip as marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "mower_map_service_lines";
        marker.id = static_cast<int>(markerArray.markers.size());
        marker.frame_locked = true;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.color = colors[0];
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;

        // Add the points to the line strip
        for (auto &point: path) {

            geometry_msgs::Point vpt;
            vpt.x = point.x;
            vpt.y = point.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start and end
        if (!path.empty()) {
            visualization_msgs::Marker marker{};

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color = colors[0];
            marker.pose.position.x = path.front().x;
            marker.pose.position.y = path.front().y;
            marker.scale.x = 0.1;
            marker.scale.y = marker.scale.z = 0.1;
            markerArray.markers.push_back(marker);
        }
    }


    // keep track of the color used last, so that we can use a new one for each path
    uint32_t cidx = 0;

    // Walk through the paths we send to the navigation stack
    for (auto &path: planning_result.paths) {
        // Each group gets a single line strip as marker
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

        // Add the points to the line strip
        for (auto &point: path.path.poses) {

            geometry_msgs::Point vpt;
            vpt.x = point.pose.position.x;
            vpt.y = point.pose.position.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start
        if (!path.path.poses.empty()) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.color = colors[cidx];
            marker.pose = path.path.poses.front().pose;
            marker.scale.x = 0.2;
            marker.scale.y = marker.scale.z = 0.05;
            markerArray.markers.push_back(marker);
        }

        // New color for a new path
        cidx = (cidx + 1) % colors.size();
    }
}

void traverse_from_left(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: contours) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_left(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

void traverse_from_right(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: boost::adaptors::reverse(contours)) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_right(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

slic3r_coverage_planner::Path determinePathForOutline(std_msgs::Header &header, Slic3r::Polygon &outline_poly, Slic3r::Polygons &group, bool isObstacle, Point *areaLastPoint) {
    slic3r_coverage_planner::Path path;
    path.is_outline = true;
    path.path.header = header;

    Point lastPoint;
    bool is_first_point = true;
    for (int i = 0; i < group.size(); i++) {
        auto points = group[i].equally_spaced_points(scale_(0.1));
        if (points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << points.size() << " points");

        if (!is_first_point) {
            // Find a good transition point between the loops.
            // It should be close to the last split point, so that we don't need to traverse a lot.

            // Find the point in the current poly which is closest to the last point of the last group
            // (which is the next inner poly from this point of view).
            const auto last_x = unscale(lastPoint.x);
            const auto last_y = unscale(lastPoint.y);
            double min_distance = INFINITY;
            int closest_idx = 0;
            for (int idx = 0; idx < points.size(); ++idx) {
                const auto &pt = points[idx];
                const auto pt_x = unscale(pt.x);
                const auto pt_y = unscale(pt.y);
                double distance = sqrt((pt_x - last_x) * (pt_x - last_x) + (pt_y - last_y) * (pt_y - last_y));
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_idx = idx;
                }
            }

            // In order to smooth the transition we skip some points (think spiral movement of the mower).
            // Check, that the skip did not break the path (cross the outer poly during transition).
            // If it's fine, use the smoothed path, otherwise use the shortest point to split.
            int smooth_transition_idx = (closest_idx + 3) % points.size();

            const Polygon *next_outer_poly;
            if (i < group.size() - 1) {
                next_outer_poly = &group[i + 1];
            } else {
                // we are in the outermost line, use outline for collision check
                next_outer_poly = &outline_poly;
            }
            Line connection(points[smooth_transition_idx], lastPoint);
            Point intersection_pt{};
            if (next_outer_poly->intersection(connection, &intersection_pt)) {
                // intersection, we need to transition at closest point
                smooth_transition_idx = closest_idx;
            }

            if (smooth_transition_idx > 0) {
                std::rotate(points.begin(), points.begin() + smooth_transition_idx, points.end());
            }
        }

        for (auto &pt: points) {
            if (is_first_point) {
                lastPoint = pt;
                is_first_point = false;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point

            // Direction for obstacle needs to be inversed compared to area outline, because we will reverse the point order later.
            auto dir = isObstacle ? lastPoint - pt : pt - lastPoint;

            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint.x);
            pose.pose.position.y = unscale(lastPoint.y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
            lastPoint = pt;
        }
    }

    if (is_first_point) {
        // there wasn't any usable point, so return the empty path
        return path;
    }

    // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation = path.path.poses.back().pose.orientation;
    pose.pose.position.x = unscale(lastPoint.x);
    pose.pose.position.y = unscale(lastPoint.y);
    pose.pose.position.z = 0;
    path.path.poses.push_back(pose);

    if (areaLastPoint != nullptr) {
        *areaLastPoint = lastPoint;
    }

    return path;
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
    const int inner_loop_number = loop_number - req.outline_overlap_count;


    Polygons gaps;

    Polygons last = expoly;
    Polygons inner = last;
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
            if (i <= inner_loop_number) {
                inner = last;
            }

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

        if (!req.skip_area_outline) {
            traverse_from_right(contours[0], area_outlines);
        }

        if (!req.skip_obstacle_outlines) {
            for (auto &hole: holes) {
                traverse_from_left(hole, obstacle_outlines);
            }
            for (auto &obstacle_group: obstacle_outlines) {
                for (auto &poly: obstacle_group) {
                    std::reverse(poly.points.begin(), poly.points.end());
                }
            }
        }
    }


    if (!req.skip_fill) {
        ExPolygons expp = union_ex(inner);


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
            fill->dont_adjust = true;
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
    }


    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;

    /**
     * Some postprocessing is done here. Until now we just have polygons (just points), but the ROS
     * navigation stack requires an orientation for each of those points as well.
     *
     * In order to achieve this, we split the polygon at some point to make it into a line with start and end.
     * Then we can calculate the orientation at each point by looking at the connection line between two points.
     */

    Point areaLastPoint;
    for (auto &group: area_outlines) {
        auto path = determinePathForOutline(header, outline_poly, group, false, &areaLastPoint);
        if (!path.path.poses.empty()) {
            res.paths.push_back(path);
        }
    }

    // The order for 3d printing seems to be to sweep across the X and then up the Y axis
    // which is very inefficient for a mower. Order the holes by distance to the previous end-point instead.
    std::vector<Slic3r::Polygons> ordered_obstacle_outlines;
    if (obstacle_outlines.size() > 0) {
        // If no prev point set to the first point in first obstacle
        // Note: back() polygon is the first (outer) loop
        auto prev_point = area_outlines.size() > 0 ? &areaLastPoint :
            &obstacle_outlines.front().back().points.front();

        while (obstacle_outlines.size()) {
            // Sort be desc distance then pop closest outline from the back of the vector
            std::sort(obstacle_outlines.begin(), obstacle_outlines.end(),
                      [prev_point](Slic3r::Polygons &a, Slic3r::Polygons &b) {
                          // Note: back() polygon is the first (outer) loop
                          auto a_firstPoint = a.back().points.front();
                          double distance_a = sqrt(
                                  (a_firstPoint.x - prev_point->x) * (a_firstPoint.x - prev_point->x) +
                                  (a_firstPoint.y - prev_point->y) * (a_firstPoint.y - prev_point->y)
                          );
                          auto b_firstPoint = b.back().points.front();
                          double distance_b = sqrt(
                                  (b_firstPoint.x - prev_point->x) * (b_firstPoint.x - prev_point->x) +
                                  (b_firstPoint.y - prev_point->y) * (b_firstPoint.y - prev_point->y)
                          );
                          return distance_a >= distance_b;
                      });
            ordered_obstacle_outlines.push_back(obstacle_outlines.back());
            obstacle_outlines.pop_back();
            // Note: front() polygon is the last (inner) loop
            prev_point = &ordered_obstacle_outlines.back().front().points.back();
        }
    }

    // At this point, the obstacles outlines are still "the wrong way" (i.e. inner first, then outer ...),
    // this is intentional, because then it's easier to find good traversal points.
    // In order to make the mower approach the obstacle, we will reverse the path later.
    for (auto &group: ordered_obstacle_outlines) {
        // Reverse here to make the mower approach the obstacle instead of starting close to the obstacle
        auto path = determinePathForOutline(header, outline_poly, group, true, nullptr);
        if (!path.path.poses.empty()) {
            std::reverse(path.path.poses.begin(), path.path.poses.end());
            res.paths.push_back(path);
        }
    }

    if (!req.skip_fill) {
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

            // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);

            res.paths.push_back(path);
        }
    }

    if (visualize_plan) {
        visualization_msgs::MarkerArray arr;
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = -1;
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::DELETEALL;
            arr.markers.push_back(marker);
        }
        createMarkers(req, res, arr);
        marker_array_publisher.publish(arr);
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
