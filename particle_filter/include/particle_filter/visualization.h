#pragma once

#include <visualization_msgs/MarkerArray.h>
#include "pose.h"

namespace visualization {
void DrawPose(const util::Pose& pose, const std::string& frame_id,
              const std::string& ns, const float r, const float g,
              const float b, const float alpha,
              visualization_msgs::MarkerArray* arr) {
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = arr->markers.size();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.tra.x();
    marker.pose.position.y = pose.tra.y();
    marker.pose.position.z = 0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = alpha;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    arr->markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = arr->markers.size();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start;
    start.x = pose.tra.x();
    start.y = pose.tra.y();
    geometry_msgs::Point end;
    const Eigen::Vector2f delta(math_util::Cos(pose.rot) * 0.4f,
                                math_util::Sin(pose.rot) * 0.4f);
    end.x = pose.tra.x() + delta.x();
    end.y = pose.tra.y() + delta.y();
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = 0.02;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = alpha;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    arr->markers.push_back(marker);
  }
}
}  // namespace visualization