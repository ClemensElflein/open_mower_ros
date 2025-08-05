#!/usr/bin/env python3
import rospy
import rosbag
from mower_map.msg import MapArea
from geometry_msgs.msg import Point32
from slic3r_coverage_planner.srv import PlanPath, PlanPathRequest, PlanPathResponse
import svgwrite
import svgwrite.shapes
from typing import List

colors = ['#e6261f', '#eb7532', '#f7d038', '#a3e048', '#49da9a', '#34bbe6', '#4355db', '#d23be7', '#000000']

def read_areas_from_map_bag(filename) -> List[MapArea]:
  areas = []
  with rosbag.Bag(filename) as bag:
    for area in bag.read_messages('mowing_areas'):
      areas.append(area.message)
  return areas

def bounding_box(points: List[Point32]):
  min_x = max_x = points[0].x
  min_y = max_y = points[0].y

  for p in points[1:]:
    if p.x < min_x:
      min_x = p.x
    elif p.x > max_x:
      max_x = p.x

    if p.y < min_y:
      min_y = p.y
    elif p.y > max_y:
      max_y = p.y

  return (min_x, min_y, max_x - min_x, max_y - min_y)

def convert_point(point: Point32):
  return (point.x, point.y)

def convert_points(points: List[Point32]):
  return map(lambda p: (p.x, p.y), points)

if __name__ == "__main__":
  areas = read_areas_from_map_bag('map.bag')
  area = areas[0]

  rospy.init_node('planner_svg', anonymous=True)
  rospy.wait_for_service('/slic3r_coverage_planner/plan_path')
  planner = rospy.ServiceProxy('/slic3r_coverage_planner/plan_path', PlanPath)

  req = PlanPathRequest()
  req.angle = 0
  req.outline_count = 3
  req.outline_overlap_count = 0
  req.outline = area.area
  req.holes = area.obstacles
  req.fill_type = PlanPathRequest.FILL_LINEAR
  req.outer_offset = 0.05
  req.distance = 0.13

  res: PlanPathResponse = planner(req)

  svg = svgwrite.Drawing('map.svg', debug=False, transform='scale(1,-1)')
  svg.viewbox(*bounding_box(area.area.points))
  svg.add(svgwrite.shapes.Polygon(convert_points(area.area.points), fill='none', stroke='green', stroke_width=0.01))
  for obstacle in area.obstacles:
    svg.add(svgwrite.shapes.Polygon(convert_points(obstacle.points), fill='none', stroke='red', stroke_width=0.01))

  color_idx = 0
  prev_point = None
  for path in res.paths:
    color = colors[color_idx]
    color_idx = (color_idx + 1) % len(colors)
    if prev_point is not None:
      svg.add(svgwrite.shapes.Line(start=convert_point(path.path.poses[0].pose.position), end=convert_point(prev_point), stroke=color, stroke_dasharray='0.04', stroke_width=0.01))
    prev_point = path.path.poses[-1].pose.position
    svg.add(svgwrite.shapes.Polyline(convert_points(map(lambda x: x.pose.position, path.path.poses)), fill='none', stroke=color, stroke_width=0.01))
    svg.add(svgwrite.shapes.Circle(convert_point(path.path.poses[0].pose.position), r=0.05, fill=color))
    svg.add(svgwrite.shapes.Circle(convert_point(path.path.poses[-1].pose.position), r=0.05, fill='white', stroke=color, stroke_width=0.03))

  svg.save()
