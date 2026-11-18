#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class SauvcArenaSimNode : public rclcpp::Node {
public:
  SauvcArenaSimNode() 
    : Node("sauvc_arena_sim_node"), 
      x_(0.7), y_(0.0), z_(1.2), roll_(0.0), pitch_(0.0), yaw_(0.0) 
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sauvc/arena_markers", 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/sauvc/cmd_vel", 10, std::bind(&SauvcArenaSimNode::cmdVelCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(50ms, std::bind(&SauvcArenaSimNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "SAUVC 3D Vectored 6-DoF Simulation Node running!");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vz_ = msg->linear.z;
    wx_ = msg->angular.x; // Roll rate
    wy_ = msg->angular.y; // Pitch rate
    wz_ = msg->angular.z; // Yaw rate
  }

  void timerCallback() {
    double dt = 0.05;

    // 1. Update 3D orientation (roll, pitch, yaw)
    roll_  += wx_ * dt;
    pitch_ += wy_ * dt;
    yaw_   += wz_ * dt;

    // Bound roll and pitch for underwater stability
    if (roll_ > M_PI) roll_ -= 2.0 * M_PI;
    if (roll_ < -M_PI) roll_ += 2.0 * M_PI;
    if (pitch_ > M_PI) pitch_ -= 2.0 * M_PI;
    if (pitch_ < -M_PI) pitch_ += 2.0 * M_PI;
    if (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    if (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    // 2. Compute 3D rotation matrix
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    tf2::Matrix3x3 R(q);

    // 3. Transform body-frame velocities [vx, vy, vz] to world coordinates
    double dx_world = R[0][0] * vx_ + R[0][1] * vy_ + R[0][2] * vz_;
    double dy_world = R[1][0] * vx_ + R[1][1] * vy_ + R[1][2] * vz_;
    double dz_world = R[2][0] * vx_ + R[2][1] * vy_ + R[2][2] * vz_;

    x_ += dx_world * dt;
    y_ += dy_world * dt;
    z_ += dz_world * dt;

    // Clamp inside pool bounds: x [0.1, 24.9], y [-7.8, 7.8], z [0.15, 1.95]
    if (x_ < 0.1) x_ = 0.1;
    if (x_ > 24.9) x_ = 24.9;
    if (y_ < -7.8) y_ = -7.8;
    if (y_ > 7.8) y_ = 7.8;
    if (z_ < 0.15) z_ = 0.15;
    if (z_ > 1.95) z_ = 1.95;

    // Broadcast base_link TF relative to world
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = z_;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    // Publish detailed visual labels and markers
    publishArenaMarkers();
  }

  void publishArenaMarkers() {
    visualization_msgs::msg::MarkerArray msg;
    rclcpp::Time now = this->now();

    // 1. DYNAMIC AUV ROBOT MARKERS (Rendered in 'world' frame)
    tf2::Quaternion q_rot;
    q_rot.setRPY(roll_, pitch_, yaw_);
    tf2::Matrix3x3 R(q_rot);

    // AUV Main Hull
    visualization_msgs::msg::Marker auv_hull;
    auv_hull.header.frame_id = "world";
    auv_hull.header.stamp = now;
    auv_hull.ns = "auv_visual";
    auv_hull.id = 0;
    auv_hull.type = visualization_msgs::msg::Marker::CYLINDER;
    auv_hull.action = visualization_msgs::msg::Marker::ADD;
    auv_hull.pose.position.x = x_;
    auv_hull.pose.position.y = y_;
    auv_hull.pose.position.z = z_;
    tf2::Quaternion q_cyl_local;
    q_cyl_local.setRPY(0, 1.5707963, 0);
    tf2::Quaternion q_hull = q_rot * q_cyl_local;
    auv_hull.pose.orientation.x = q_hull.x();
    auv_hull.pose.orientation.y = q_hull.y();
    auv_hull.pose.orientation.z = q_hull.z();
    auv_hull.pose.orientation.w = q_hull.w();
    auv_hull.scale.x = 0.28;
    auv_hull.scale.y = 0.28;
    auv_hull.scale.z = 0.65;
    auv_hull.color.r = 1.0;
    auv_hull.color.g = 0.8;
    auv_hull.color.b = 0.0;
    auv_hull.color.a = 1.0;
    msg.markers.push_back(auv_hull);

    // AUV Outer Frame Box
    visualization_msgs::msg::Marker auv_frame;
    auv_frame.header.frame_id = "world";
    auv_frame.header.stamp = now;
    auv_frame.ns = "auv_visual";
    auv_frame.id = 1;
    auv_frame.type = visualization_msgs::msg::Marker::CUBE;
    auv_frame.action = visualization_msgs::msg::Marker::ADD;
    auv_frame.pose.position.x = x_;
    auv_frame.pose.position.y = y_;
    auv_frame.pose.position.z = z_;
    auv_frame.pose.orientation.x = q_rot.x();
    auv_frame.pose.orientation.y = q_rot.y();
    auv_frame.pose.orientation.z = q_rot.z();
    auv_frame.pose.orientation.w = q_rot.w();
    auv_frame.scale.x = 0.65;
    auv_frame.scale.y = 0.40;
    auv_frame.scale.z = 0.30;
    auv_frame.color.r = 0.15;
    auv_frame.color.g = 0.15;
    auv_frame.color.b = 0.20;
    auv_frame.color.a = 0.85;
    msg.markers.push_back(auv_frame);

    // AUV Red Kill Switch
    tf2::Vector3 kill_local(-0.22, 0.0, 0.18);
    tf2::Vector3 kill_world = tf2::Vector3(x_, y_, z_) + (R * kill_local);
    visualization_msgs::msg::Marker auv_kill;
    auv_kill.header.frame_id = "world";
    auv_kill.header.stamp = now;
    auv_kill.ns = "auv_visual";
    auv_kill.id = 2;
    auv_kill.type = visualization_msgs::msg::Marker::CYLINDER;
    auv_kill.action = visualization_msgs::msg::Marker::ADD;
    auv_kill.pose.position.x = kill_world.x();
    auv_kill.pose.position.y = kill_world.y();
    auv_kill.pose.position.z = kill_world.z();
    auv_kill.pose.orientation.x = q_rot.x();
    auv_kill.pose.orientation.y = q_rot.y();
    auv_kill.pose.orientation.z = q_rot.z();
    auv_kill.pose.orientation.w = q_rot.w();
    auv_kill.scale.x = 0.08;
    auv_kill.scale.y = 0.08;
    auv_kill.scale.z = 0.10;
    auv_kill.color.r = 1.0;
    auv_kill.color.g = 0.05;
    auv_kill.color.b = 0.05;
    auv_kill.color.a = 1.0;
    msg.markers.push_back(auv_kill);

    // 8 Vectored Thrusters
    struct ThrusterPos { double x; double y; double z; double pitch; double yaw; };
    std::vector<ThrusterPos> thrusters = {
      { 0.24,  0.20,  0.0, 0.0,  0.785},
      { 0.24, -0.20,  0.0, 0.0, -0.785},
      {-0.24,  0.20,  0.0, 0.0,  2.356},
      {-0.24, -0.20,  0.0, 0.0, -2.356},
      { 0.16,  0.20,  0.09, 1.5708, 0.0},
      { 0.16, -0.20,  0.09, 1.5708, 0.0},
      {-0.16,  0.20,  0.09, 1.5708, 0.0},
      {-0.16, -0.20,  0.09, 1.5708, 0.0}
    };

    int t_id = 100;
    for (const auto& tp : thrusters) {
      tf2::Vector3 th_local(tp.x, tp.y, tp.z);
      tf2::Vector3 th_world = tf2::Vector3(x_, y_, z_) + (R * th_local);

      tf2::Quaternion q_t_local;
      q_t_local.setRPY(0.0, tp.pitch, tp.yaw);
      tf2::Quaternion q_t_world = q_rot * q_t_local;

      visualization_msgs::msg::Marker th;
      th.header.frame_id = "world";
      th.header.stamp = now;
      th.ns = "auv_thrusters";
      th.id = t_id++;
      th.type = visualization_msgs::msg::Marker::CYLINDER;
      th.action = visualization_msgs::msg::Marker::ADD;
      th.pose.position.x = th_world.x();
      th.pose.position.y = th_world.y();
      th.pose.position.z = th_world.z();
      th.pose.orientation.x = q_t_world.x();
      th.pose.orientation.y = q_t_world.y();
      th.pose.orientation.z = q_t_world.z();
      th.pose.orientation.w = q_t_world.w();
      th.scale.x = 0.07;
      th.scale.y = 0.07;
      th.scale.z = 0.10;
      th.color.r = 0.1;
      th.color.g = 0.85;
      th.color.b = 1.0;
      th.color.a = 1.0;
      msg.markers.push_back(th);
    }

    // AUV Text Header Label
    visualization_msgs::msg::Marker auv_label;
    auv_label.header.frame_id = "world";
    auv_label.header.stamp = now;
    auv_label.ns = "auv_visual";
    auv_label.id = 3;
    auv_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    auv_label.action = visualization_msgs::msg::Marker::ADD;
    auv_label.pose.position.x = x_;
    auv_label.pose.position.y = y_;
    auv_label.pose.position.z = z_ + 0.45;
    auv_label.scale.z = 0.35;
    auv_label.color.r = 0.2;
    auv_label.color.g = 1.0;
    auv_label.color.b = 0.3;
    auv_label.color.a = 1.0;
    auv_label.text = ">> 6-DoF VECTORED AUV <<";
    msg.markers.push_back(auv_label);

    // 2. ARENA TEXT LABELS
    int id = 10;
    auto addText = [&](const std::string& text, double px, double py, double pz, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp = now;
      m.ns = "labels";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = px;
      m.pose.position.y = py;
      m.pose.position.z = pz;
      m.scale.z = 0.35;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0;
      m.text = text;
      msg.markers.push_back(m);
    };

    addText("START ZONE (140x140 cm)", 0.7, 0.0, 2.3, 1.0f, 0.9f, 0.2f);
    addText("QUALIFICATION GATE (150 cm)", 10.0, -4.0, 2.3, 1.0f, 0.5f, 0.0f);
    addText("OBSTACLE FLARE (Orange 15cm)", 6.0, 0.35, 2.3, 1.0f, 0.4f, 0.0f);
    addText("MAIN NAV GATE (150x100 cm)", 16.0, 0.0, 1.4, 0.2f, 1.0f, 0.3f);
    addText("BLUE DRUM (30 Pts)", 19.5, -0.8, 0.7, 0.2f, 0.5f, 1.0f);
    addText("RED DRUM + 45kHz PINGER (50 Pts)", 19.5, -2.0, 0.7, 1.0f, 0.2f, 0.2f);
    addText("TASK 4 FLARES (R-Y-B)", 13.0, 4.0, 1.3, 1.0f, 0.8f, 0.1f);

    marker_pub_->publish(msg);
  }

  double x_, y_, z_;
  double roll_, pitch_, yaw_;
  double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
  double wx_ = 0.0, wy_ = 0.0, wz_ = 0.0;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SauvcArenaSimNode>());
  rclcpp::shutdown();
  return 0;
}
