#include "safety_shield/rviz_marker.h"

namespace safety_shield {

RvizMarker::RvizMarker(ros::Publisher robot_visu_pub, ros::Publisher human_cylinder_visu_pub, ros::Publisher human_reach_visu_pub_P, ros::Publisher human_reach_visu_pub_V, ros::Publisher human_reach_visu_pub_A):
    robot_visu_pub_(robot_visu_pub),
    human_cylinder_visu_pub_(human_cylinder_visu_pub),
    human_reach_visu_pub_P_(human_reach_visu_pub_P),
    human_reach_visu_pub_V_(human_reach_visu_pub_V),
    human_reach_visu_pub_A_(human_reach_visu_pub_A)
{
}


void RvizMarker::robot_callback(const custom_robot_msgs::CapsuleArray* data) {
  try {
    // Check if capsule size stayed the same.
    if (this->robot_markers_.markers.size() != 3*data->capsules.size()) {
      this->robot_markers_.markers.clear();
      createPoints(this->robot_markers_, 3*data->capsules.size(), visualization_msgs::Marker::CYLINDER, RvizMarker::ROBOT);
    } 
    createCapsules(this->robot_markers_, data);
    this->robot_visu_pub_.publish(this->robot_markers_);
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in RvizMarker::robot_callback: " << exc.what());
  }
}

void RvizMarker::advanced_robot_callback(const custom_robot_msgs::StartGoalCapsuleArray* data) {
  try {
    custom_robot_msgs::CapsuleArray* cap(new custom_robot_msgs::CapsuleArray());
    cap->capsules = data->capsules;
    cap->header = data->header;
    this->robot_callback(cap);
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in RvizMarker::robot_callback: " << exc.what());
  }
}

void RvizMarker::human_cylinder_callback(const custom_robot_msgs::PolycapsuleArray* data) {
  this->human_cylinder_markers_.markers.clear();
  createPoints(this->human_cylinder_markers_, 3*data->polycapsules.size(), visualization_msgs::Marker::CYLINDER, RvizMarker::ROBOT);
  auto marker = this->human_cylinder_markers_.markers.begin();
  for(auto set = data->polycapsules.begin(); set != data->polycapsules.end(); set++) {
    if(set->polygon.size() == 1) {
      //first circle
      createSphere(set->polygon.front().p, set->radius, data->header.stamp, *marker);
      marker++;
      //second circle
      createSphere(set->polygon.front().q, set->radius, data->header.stamp, *marker);
      marker++;
      //line
      createCylinder(set->polygon.front(), set->radius, data->header.stamp, *marker);
      marker++;
    }
    else if(set->polygon.size() > 1) {
      // NOT IMPLEMENTED YET
      /*
      // edges
      for(auto seg = set->polygon.begin(); seg != set->polygon.end(); seg++) {
        marker->type = visualization_msgs::Marker::SPHERE;
        marker->pose.position = seg->p;
        marker->scale.x = 2*set->radius;
        marker->scale.y = 2*set->radius;
        marker->scale.z = 2*set->radius;
        marker->header.stamp = data->human_sets.header.stamp;
        marker++;
      }

      // sides
      marker->type = visualization_msgs::Marker::LINE_LIST;
      marker->points = polygon_helpers::fromSegmentToPoints(set->polygon);
      marker->scale.x = 2*set->radius;
      marker->header.stamp = data->human_sets.header.stamp;
      marker++;
 
      //inside
      auto ck_polygon = polygon_helpers::getClockwisePolygon(set->polygon);
      marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
      auto info = polygon_helpers::triangularisation(ck_polygon);
      marker->points = info;
      marker->scale.x = 1;
      marker->scale.y = 1;
      marker->scale.z = 1;
      marker->header.stamp = data->human_sets.header.stamp;
      marker++;
      */
    }
  }
}

void RvizMarker::human_reach_callback_p(const custom_robot_msgs::CapsuleArray* data) {
  try {
    human_reach_calc(data, this->human_reach_markers_P_);
    this->human_reach_visu_pub_P_.publish(this->human_reach_markers_P_);
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in RvizMarker::human_reach_callback_p: " << exc.what());
  }
}
void RvizMarker::human_reach_callback_v(const custom_robot_msgs::CapsuleArray* data) {
  try {
    human_reach_calc(data, this->human_reach_markers_V_);
    this->human_reach_visu_pub_V_.publish(this->human_reach_markers_V_);
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in RvizMarker::human_reach_callback_v: " << exc.what());
  }
}
void RvizMarker::human_reach_callback_a(const custom_robot_msgs::CapsuleArray* data) {
  human_reach_calc(data, this->human_reach_markers_A_);
  this->human_reach_visu_pub_A_.publish(this->human_reach_markers_A_);
}

void RvizMarker::human_reach_calc(const custom_robot_msgs::CapsuleArray* data, 
    visualization_msgs::MarkerArray& human_reach_markers) {
  try {
    // Check if capsule size stayed the same.
    if (human_reach_markers.markers.size() != 3*data->capsules.size()) {
      human_reach_markers.markers.clear();
      createPoints(human_reach_markers, 3*data->capsules.size(), visualization_msgs::Marker::CYLINDER, RvizMarker::HUMAN_REACH);
    } 
    createCapsules(human_reach_markers, data);
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in RvizMarker::human_reach_callback_v: " << exc.what());
  }
}


void RvizMarker::createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, 
    int color_type) {
  int prev_size = markers.markers.size();
  for(int i = 0; i < nb_points_to_add; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.ns = "shapes";
    marker.id = prev_size+i;
    marker.type = shape_type;
    if(color_type == RvizMarker::ROBOT) {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }
    else if (color_type == RvizMarker::HUMAN_CYLINDER) {
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
    } else if (color_type == RvizMarker::HUMAN_REACH) {
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }   
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    markers.markers.push_back(marker);
  }
}


void RvizMarker::createCapsules(visualization_msgs::MarkerArray& markers, 
    const custom_robot_msgs::CapsuleArray* capsules) { 
  auto marker = markers.markers.begin();
  for(auto cap = capsules->capsules.begin(); cap != capsules->capsules.end(); cap++) {
    geometry_msgs::Point p1 = cap->segment.p;
    geometry_msgs::Point p2 = cap->segment.q;
    //first circle
    createSphere(p1, cap->radius, capsules->header.stamp, *marker);
    marker++;
    //second circle
    createSphere(p2, cap->radius, capsules->header.stamp, *marker);
    marker++;
    //middle cylinder
    createCylinder(cap->segment, cap->radius, capsules->header.stamp, *marker);
    marker++;
  }
}

void RvizMarker::createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position = pos;
  marker.scale.x = 2*radius;
  marker.scale.y = 2*radius;
  marker.scale.z = 2*radius;
  marker.header.stamp = stamp;
}


void RvizMarker::createCylinder(const custom_robot_msgs::Segment& seg, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
  double p1x = seg.p.x;
  double p1y = seg.p.y;
  double p1z = seg.p.z;
  double p2x = seg.q.x;
  double p2y = seg.q.y;
  double p2z = seg.q.z;
  double v2_x = (p2x-p1x);
  double v2_y = (p2y-p1y);
  double v2_z = (p2z-p1z);
  double norm = sqrt(pow(v2_x, 2) + pow(v2_y, 2) + pow(v2_z, 2));
  if(norm > 1e-6) {
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = (p1x + p2x)/2;
    marker.pose.position.y = (p1y + p2y)/2;
    marker.pose.position.z = (p1z + p2z)/2;
    // Rotate z axis vector to direction vector according to https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another/1171995#1171995
    double a_x = -v2_y/norm;
    double a_y = v2_x/norm;
    double a_z = 0;
    double a_w = 1 + v2_z/norm;
    double norm_q = sqrt(pow(a_w, 2) + pow(a_x, 2) + pow(a_y, 2) + pow(a_z, 2));
    marker.pose.orientation.w = a_w/norm_q;
    marker.pose.orientation.x = a_x/norm_q;
    marker.pose.orientation.y = a_y/norm_q;
    marker.pose.orientation.z = a_z/norm_q;
    marker.scale.z = norm;
    marker.scale.y = 2*radius;
    marker.scale.x = 2*radius;
  }
  else{
    marker.scale.x = 0;
    marker.scale.y = 0;
    marker.scale.z = 0;
  }
  marker.header.stamp = stamp;
}

} // namespace safety_shield