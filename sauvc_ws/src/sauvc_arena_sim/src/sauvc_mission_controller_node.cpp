#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

enum class MissionState {
  STEP1_SUBMERGE,
  STEP2_AVOID_ORANGE_FLARE,
  STEP2_PASS_GATE,
  STEP3_PICK_BALL_FLARE_1,   // Flare 1: Blue Flare at (11.5m, 2.2m)
  STEP4_DROP_BALL_DRUM_1,   // Drum 1: Blue Drum at (23.0m, 4.5m)
  STEP5_PICK_BALL_FLARE_2,   // Flare 2: Red Flare at (14.0m, 3.5m)
  STEP6_DROP_BALL_DRUM_2,   // Drum 2: Red Pinger Drum at (23.0m, 1.5m)
  STEP7_PICK_BALL_FLARE_3,   // Flare 3: Yellow Flare at (13.0m, -3.5m)
  STEP8_DROP_BALL_DRUM_3,   // Drum 3: Red Drum at (23.0m, -1.5m)
  STEP9_SURFACE_FINISH
};

class SauvcMissionControllerNode : public rclcpp::Node {
public:
  SauvcMissionControllerNode() 
    : Node("sauvc_mission_controller_node"),
      x_(0.7), y_(2.0), z_(1.8), roll_(0.0), pitch_(0.0), yaw_(0.0),
      current_state_(MissionState::STEP1_SUBMERGE),
      state_timer_(0.0),
      score_(0),
      ball1_picked_(false), ball1_dropped_(false),
      ball2_picked_(false), ball2_dropped_(false),
      ball3_picked_(false), ball3_dropped_(false)
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sauvc/arena_markers", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/sauvc/mission_status", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Camera Image & Info Publishers for Front and Downward Underwater Cameras
    front_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sauvc/front_camera/image_raw", 10);
    front_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/sauvc/front_camera/camera_info", 10);
    down_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sauvc/down_camera/image_raw", 10);
    down_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/sauvc/down_camera/camera_info", 10);

    // Simulation control subscriptions (Start, Stop/Pause, Reset, and Manual Teleop)
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/sauvc/cmd_vel", 10);
    set_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/sauvc/set_entity_state");
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/sauvc/cmd_vel", 10, std::bind(&SauvcMissionControllerNode::cmdVelCallback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/sauvc/sim/start", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          is_running_ = true;
          autonomous_mode_ = true;
          RCLCPP_INFO(this->get_logger(), "Simulation STARTED / RESUMED");
        }
      });

    stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/sauvc/sim/stop", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          is_running_ = false;
          vx_ = 0.0; vy_ = 0.0; vz_ = 0.0;
          wx_ = 0.0; wy_ = 0.0; wz_ = 0.0;
          status_text_ = "SIMULATION PAUSED";
          RCLCPP_INFO(this->get_logger(), "Simulation PAUSED / STOPPED");
        }
      });

    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/sauvc/sim/reset", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          resetMission();
          RCLCPP_INFO(this->get_logger(), "Simulation RESET TO START");
        }
      });

    control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/sauvc/control_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "manual") {
          autonomous_mode_ = false;
          RCLCPP_INFO(this->get_logger(), "Switched to MANUAL CONTROL mode");
        } else if (msg->data == "auto") {
          autonomous_mode_ = true;
          RCLCPP_INFO(this->get_logger(), "Switched to AUTONOMOUS mode");
        }
      });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(50ms, std::bind(&SauvcMissionControllerNode::updateLoop, this));
    RCLCPP_INFO(this->get_logger(), "SAUVC 9-Step Autonomous Mission Controller Running!");
  }

  void resetMission() {
    x_ = 0.7; y_ = 2.0; z_ = 1.8;
    roll_ = 0.0; pitch_ = 0.0; yaw_ = 0.0;
    vx_ = 0.0; vy_ = 0.0; vz_ = 0.0;
    wx_ = 0.0; wy_ = 0.0; wz_ = 0.0;
    current_state_ = MissionState::STEP1_SUBMERGE;
    state_timer_ = 0.0;
    score_ = 0;
    ball1_picked_ = false; ball1_dropped_ = false;
    ball2_picked_ = false; ball2_dropped_ = false;
    ball3_picked_ = false; ball3_dropped_ = false;
    ball1_drop_start_time_ = 0.0;
    ball2_drop_start_time_ = 0.0;
    ball3_drop_start_time_ = 0.0;
    gripper_jaw_pos_ = 0.28;
    is_running_ = true;
    autonomous_mode_ = true;
    status_text_ = "1) Starting Point: Submerging into pool (depth z -> 0.82m)";
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (std::abs(msg->linear.x) > 0.01 || std::abs(msg->angular.z) > 0.01) {
      autonomous_mode_ = false;
      vx_ = msg->linear.x;
      vy_ = msg->linear.y;
      vz_ = msg->linear.z;
      wx_ = msg->angular.x;
      wy_ = msg->angular.y;
      wz_ = msg->angular.z;
    }
  }

  void updateLoop() {
    double dt = 0.05;

    if (!is_running_) {
      // Simulation paused: publish static state and camera views without movement
      publishMissionDisplays();
      renderAndPublishCameraViews();
      return;
    }

    state_timer_ += dt;

    if (autonomous_mode_) {
      runMissionFSM(dt);
    }

    // 1. Integrate 3D angular velocities
    roll_  += wx_ * dt;
    pitch_ += wy_ * dt;
    yaw_   += wz_ * dt;

    while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    // 2. 3D Rotation Matrix for Body-to-World transformation
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    tf2::Matrix3x3 R(q);

    double dx_world = R[0][0] * vx_ + R[0][1] * vy_ + R[0][2] * vz_;
    double dy_world = R[1][0] * vx_ + R[1][1] * vy_ + R[1][2] * vz_;
    double dz_world = R[2][0] * vx_ + R[2][1] * vy_ + R[2][2] * vz_;

    x_ += dx_world * dt;
    y_ += dy_world * dt;
    z_ += dz_world * dt;

    // Safety boundary constraints
    if (x_ < 0.2) x_ = 0.2;
    if (x_ > 24.8) x_ = 24.8;
    if (y_ < -7.7) y_ = -7.7;
    if (y_ > 7.7) y_ = 7.7;
    if (z_ < 0.15) z_ = 0.15;
    if (z_ > 1.95) z_ = 1.95;

    // 3. Broadcast TF: world -> base_link
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

    // Dynamic TF for the 3 arena flare balls
    auto broadcastBallTF = [&](const std::string& child_id, bool is_picked, bool is_dropped,
                              double drop_start_time, double flare_x, double flare_y,
                              double drum_x, double drum_y) 
    {
      (void)drop_start_time;
      geometry_msgs::msg::TransformStamped tb;
      tb.header.stamp = this->now();
      tb.header.frame_id = "world";
      tb.child_frame_id = child_id;
      tb.transform.rotation.w = 1.0;

      if (!is_picked) {
        // Resting on top of the flare pole
        tb.transform.translation.x = flare_x;
        tb.transform.translation.y = flare_y;
        tb.transform.translation.z = 0.84;
      } else if (!is_dropped) {
        // Gripped tightly in robot claws: transforms 1-to-1 with vehicle motion
        tf2::Vector3 local_claw(0.44, 0.0, -0.05);
        tf2::Vector3 world_claw = tf2::Vector3(x_, y_, z_) + (R * local_claw);
        tb.transform.translation.x = world_claw.x();
        tb.transform.translation.y = world_claw.y();
        tb.transform.translation.z = world_claw.z();
      } else {
        // Released above drum: ball floats gently at target drum hover height
        tb.transform.translation.x = drum_x;
        tb.transform.translation.y = drum_y;
        tb.transform.translation.z = 0.70; // Floats stationary above the drum
      }
      tf_broadcaster_->sendTransform(tb);
    };

    broadcastBallTF("flare_blue_ball", ball1_picked_, ball1_dropped_, ball1_drop_start_time_, 11.5, 2.2, 23.0, 4.5);
    broadcastBallTF("flare_red_ball", ball2_picked_, ball2_dropped_, ball2_drop_start_time_, 14.0, 3.5, 23.0, 1.5);
    broadcastBallTF("flare_yellow_ball", ball3_picked_, ball3_dropped_, ball3_drop_start_time_, 13.0, -3.5, 23.0, -1.5);

    // 4. Determine Gripper Clamping State
    // Gripper is closed (angle = -0.32 rad left, +0.32 rad right) when clamping a ball
    // Gripper is open (angle = 0.25 rad left, -0.25 rad right) when approaching or after dropping
    bool carrying_ball = (ball1_picked_ && !ball1_dropped_) ||
                         (ball2_picked_ && !ball2_dropped_) ||
                         (ball3_picked_ && !ball3_dropped_);

    double target_jaw_angle = carrying_ball ? -0.28 : 0.28;
    // Smooth servo interpolation
    gripper_jaw_pos_ += (target_jaw_angle - gripper_jaw_pos_) * 0.15;

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = {"joint_gripper_finger_left", "joint_gripper_finger_right"};
    js.position = {gripper_jaw_pos_, -gripper_jaw_pos_};
    joint_state_pub_->publish(js);

    // Publish velocity command for Gazebo physics
    if (autonomous_mode_) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = vx_;
      twist.linear.y = vy_;
      twist.linear.z = vz_;
      twist.angular.x = wx_;
      twist.angular.y = wy_;
      twist.angular.z = wz_;
      cmd_pub_->publish(twist);
    }

    // Direct 6-DoF Entity State Update to Gazebo via SetEntityState service (fire-and-forget async)
    if (set_state_client_->service_is_ready()) {
      auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
      req->state.name = "sauvc_auv";
      req->state.reference_frame = "world";
      req->state.pose.position.x = x_;
      req->state.pose.position.y = y_;
      req->state.pose.position.z = z_;
      req->state.pose.orientation.x = q.x();
      req->state.pose.orientation.y = q.y();
      req->state.pose.orientation.z = q.z();
      req->state.pose.orientation.w = q.w();
      req->state.twist.linear.x = dx_world;
      req->state.twist.linear.y = dy_world;
      req->state.twist.linear.z = dz_world;
      req->state.twist.angular.z = wz_;
      set_state_client_->async_send_request(req);
    }

    // 5. Publish dynamic 3D elements (scoreboard, labels)
    publishMissionDisplays();

    // 6. Render and publish Front and Downward underwater camera feeds
    renderAndPublishCameraViews();
  }

  void navigateTo(double target_x, double target_y, double target_z, double target_yaw, double speed = 0.35) {
    double dx = target_x - x_;
    double dy = target_y - y_;
    double dz = target_z - z_;
    double dist = std::sqrt(dx * dx + dy * dy);

    double desired_yaw = (dist > 0.3) ? std::atan2(dy, dx) : target_yaw;
    double dyaw = desired_yaw - yaw_;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

    // Angular rate control (reduced by 50% for smooth turning)
    wz_ = 0.7 * dyaw;
    if (wz_ > 0.45) wz_ = 0.45;
    if (wz_ < -0.45) wz_ = -0.45;

    // Linear surge control (reduced by 50%)
    if (std::abs(dyaw) < 0.7) {
      vx_ = speed;
    } else {
      vx_ = speed * 0.25;
    }

    // Depth heave control (reduced by 50%)
    vz_ = 0.45 * dz;
    if (vz_ > 0.225) vz_ = 0.225;
    if (vz_ < -0.225) vz_ = -0.225;

    // Pitch & roll leveling
    wy_ = -0.8 * pitch_;
    wx_ = -0.8 * roll_;
    vy_ = 0.0;
  }

  void runMissionFSM(double dt) {
    (void)dt;
    switch (current_state_) {
      // 1) Start from starting point and get down from surface (50% reduced heave: vz = -0.15)
      case MissionState::STEP1_SUBMERGE:
        status_text_ = "1) Starting Point: Submerging into pool (depth z -> 0.82m)";
        vx_ = 0.0; vy_ = 0.0; wz_ = 0.0;
        vz_ = -0.15;
        if (z_ <= 0.82) {
          vz_ = 0.0;
          current_state_ = MissionState::STEP2_AVOID_ORANGE_FLARE;
          state_timer_ = 0.0;
        }
        break;

      // 2) Pass through the gate (Orange Flare obstacle aligned with gate at x=6, y=-1; navigate around it then cross gate at x=16, y=-1)
      case MissionState::STEP2_AVOID_ORANGE_FLARE:
        status_text_ = "2) Navigation: Clearing Orange Flare obstacle (aligning towards Gate at y=-1.0m)";
        if (x_ < 5.0) {
          // Approach towards gate line while staying clear of the orange flare obstacle
          navigateTo(5.0, 0.0, 0.6, 0.0, 0.6);
        } else if (x_ < 8.5) {
          // Clear orange obstacle flare at (6.0, -1.0) by skirting around it to y=-0.1m
          navigateTo(8.5, -0.1, 0.6, 0.0, 0.6);
        } else {
          // Line up directly with center of Navigation Gate (x=16.0m, y=-1.0m)
          navigateTo(15.2, -1.0, 0.5, 0.0, 0.6);
          if (x_ >= 14.8) {
            current_state_ = MissionState::STEP2_PASS_GATE;
            state_timer_ = 0.0;
          }
        }
        break;

      case MissionState::STEP2_PASS_GATE:
        status_text_ = "2) Navigation: Passing through 150x100cm Navigation Gate (+15 Pts)";
        navigateTo(17.5, -1.0, 0.5, 0.0, 0.6);
        if (x_ >= 17.0) {
          score_ += 15;
          current_state_ = MissionState::STEP3_PICK_BALL_FLARE_1;
          state_timer_ = 0.0;
        }
        break;

      // 3) Pick a ball from anyone flare (Flare 1: BLUE FLARE at 11.5m, 2.2m)
      case MissionState::STEP3_PICK_BALL_FLARE_1:
        status_text_ = "3) Approaching BLUE FLARE (11.5m, 2.2m) -> Gripper locking onto Ball 1";
        navigateTo(11.95, 2.2, 0.82, 3.14159, 0.6);
        if (std::sqrt(std::pow(x_ - 11.95, 2) + std::pow(y_ - 2.2, 2)) < 0.35) {
          vx_ = 0.0; vy_ = 0.0; vz_ = 0.0; wz_ = 0.0;
          if (state_timer_ > 1.2) {
            ball1_picked_ = true;
            score_ += 20;
            current_state_ = MissionState::STEP4_DROP_BALL_DRUM_1;
            state_timer_ = 0.0;
          }
        }
        break;

      // 4) Drop it in any drum (Drum 1: BLUE DRUM at 23.0m, 4.5m)
      case MissionState::STEP4_DROP_BALL_DRUM_1:
        status_text_ = "4) Transporting Ball 1 in gripper -> Hovering over BLUE DRUM (23.0m, 4.5m)";
        navigateTo(22.60, 4.5, 0.75, 0.0, 0.65);
        if (std::sqrt(std::pow(x_ - 22.60, 2) + std::pow(y_ - 4.5, 2)) < 0.35) {
          vx_ = 0.0; vy_ = 0.0; vz_ = 0.0; wz_ = 0.0;
          status_text_ = "4) Releasing Gripper -> Ball 1 floating over BLUE DRUM (+30 Pts)";
          if (state_timer_ > 0.8 && !ball1_dropped_) {
            ball1_dropped_ = true;
            ball1_drop_start_time_ = state_timer_;
            score_ += 30;
          }
          if (state_timer_ > 2.0) {
            current_state_ = MissionState::STEP5_PICK_BALL_FLARE_2;
            state_timer_ = 0.0;
          }
        }
        break;

      // 5) Pick second ball from remaining flare (Flare 2: RED FLARE at 14.0m, 3.5m)
      case MissionState::STEP5_PICK_BALL_FLARE_2:
        status_text_ = "5) Approaching RED FLARE (14.0m, 3.5m) -> Gripper locking onto Ball 2";
        navigateTo(14.45, 3.5, 0.82, 3.14159, 0.65);
        if (std::sqrt(std::pow(x_ - 14.45, 2) + std::pow(y_ - 3.5, 2)) < 0.35) {
          vx_ = 0.0; vy_ = 0.0; vz_ = 0.0; wz_ = 0.0;
          if (state_timer_ > 1.2) {
            ball2_picked_ = true;
            score_ += 20;
            current_state_ = MissionState::STEP6_DROP_BALL_DRUM_2;
            state_timer_ = 0.0;
          }
        }
        break;

      // 6) Drop it in any one remaining drum (Drum 2: RED PINGER DRUM at 23.0m, 1.5m)
      case MissionState::STEP6_DROP_BALL_DRUM_2:
        status_text_ = "6) Transporting Ball 2 in gripper -> Hovering over RED PINGER DRUM (23.0m, 1.5m)";
        navigateTo(22.60, 1.5, 0.75, 0.0, 0.65);
        if (std::sqrt(std::pow(x_ - 22.60, 2) + std::pow(y_ - 1.5, 2)) < 0.35) {
          vx_ = 0.0; vy_ = 0.0; vz_ = 0.0; wz_ = 0.0;
          status_text_ = "6) Releasing Gripper -> Ball 2 floating over RED PINGER DRUM (+50 Pts)";
          if (state_timer_ > 0.8 && !ball2_dropped_) {
            ball2_dropped_ = true;
            ball2_drop_start_time_ = state_timer_;
            score_ += 50;
          }
          if (state_timer_ > 2.0) {
            current_state_ = MissionState::STEP7_PICK_BALL_FLARE_3;
            state_timer_ = 0.0;
          }
        }
        break;

      // 7) Pick last ball from flare (Flare 3: YELLOW FLARE at 13.0m, -3.5m)
      case MissionState::STEP7_PICK_BALL_FLARE_3:
        status_text_ = "7) Approaching YELLOW FLARE (13.0m, -3.5m) -> Gripper locking onto Ball 3";
        navigateTo(13.45, -3.5, 0.82, 3.14159, 0.65);
        if (std::sqrt(std::pow(x_ - 13.45, 2) + std::pow(y_ - (-3.5), 2)) < 0.35) {
          vx_ = 0.0; vy_ = 0.0; vz_ = 0.0; wz_ = 0.0;
          if (state_timer_ > 1.2) {
            ball3_picked_ = true;
            score_ += 20;
            current_state_ = MissionState::STEP8_DROP_BALL_DRUM_3;
            state_timer_ = 0.0;
          }
        }
        break;

      // 8) Drop it any one remaining drum (Drum 3: REMAINING RED DRUM at 23.0m, -1.5m)
      case MissionState::STEP8_DROP_BALL_DRUM_3:
        status_text_ = "8) Transporting Ball 3 in gripper -> Hovering over RED DRUM (23.0m, -1.5m)";
        navigateTo(22.60, -1.5, 0.75, 0.0, 0.65);
        if (std::sqrt(std::pow(x_ - 22.60, 2) + std::pow(y_ - (-1.5), 2)) < 0.35) {
          vx_ = 0.0; vy_ = 0.0; vz_ = 0.0; wz_ = 0.0;
          status_text_ = "8) Releasing Gripper -> Ball 3 floating over RED DRUM (+10 Pts)";
          if (state_timer_ > 0.8 && !ball3_dropped_) {
            ball3_dropped_ = true;
            ball3_drop_start_time_ = state_timer_;
            score_ += 10;
          }
          if (state_timer_ > 2.0) {
            current_state_ = MissionState::STEP9_SURFACE_FINISH;
            state_timer_ = 0.0;
          }
        }
        break;

      // 9) Come towards to surface (ascent speed reduced by 50%: vz = 0.175)
      case MissionState::STEP9_SURFACE_FINISH:
        status_text_ = "9) Mission Complete: Controlled Ascent towards Surface (z -> 1.85m)";
        vx_ = 0.0; vy_ = 0.0; wz_ = 0.0;
        vz_ = 0.175;
        if (z_ >= 1.85) {
          vz_ = 0.0;
          status_text_ = "SUCCESS: ALL 3 FLARE BALLS RETRIEVED & DROPPED! TOTAL SCORE: " + std::to_string(score_) + " PTS";
        }
        break;
    }
  }

  void publishMissionDisplays() {
    visualization_msgs::msg::MarkerArray msg;
    rclcpp::Time now = this->now();

    // 1. HUD Billboard with live step and score
    visualization_msgs::msg::Marker hud;
    hud.header.frame_id = "world";
    hud.header.stamp = now;
    hud.ns = "mission_hud";
    hud.id = 1;
    hud.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    hud.action = visualization_msgs::msg::Marker::ADD;
    hud.pose.position.x = 12.5;
    hud.pose.position.y = 0.0;
    hud.pose.position.z = 4.2;
    hud.scale.z = 0.65;
    hud.color.r = 0.1f;
    hud.color.g = 1.0f;
    hud.color.b = 0.35f;
    hud.color.a = 1.0f;
    hud.text = "SAUVC 9-STEP AUTONOMOUS MISSION\n" + status_text_ + 
               "\nSCORE: " + std::to_string(score_) + " PTS | VEHICLE: 6-DoF VECTORED AUV + GRIPPER";
    msg.markers.push_back(hud);

  // 2. Dynamic 3D Ball Markers
    // Clearly shows balls on flares, gripped in robot claws, and deposited with falling drop into drums
    auto renderBall = [&](int ball_id, bool is_picked, bool is_dropped, double drop_start_time,
                         double flare_x, double flare_y, 
                         double drum_x, double drum_y) 
    {
      (void)drop_start_time;
      visualization_msgs::msg::Marker ball;
      ball.header.frame_id = "world";
      ball.header.stamp = now;
      ball.ns = "mission_balls";
      ball.id = ball_id;
      ball.type = visualization_msgs::msg::Marker::SPHERE;
      ball.action = visualization_msgs::msg::Marker::ADD;
      ball.scale.x = 0.22; // High-visibility 22cm white sphere
      ball.scale.y = 0.22;
      ball.scale.z = 0.22;
      ball.color.r = 1.0f;
      ball.color.g = 1.0f;
      ball.color.b = 1.0f;
      ball.color.a = 1.0f;

      if (!is_picked) {
        // Resting clearly on top of the flare pole at z = 0.90m
        ball.pose.position.x = flare_x;
        ball.pose.position.y = flare_y;
        ball.pose.position.z = 0.90;
      } else if (!is_dropped) {
        // Firmly held in the front gripper claws of the moving AUV: moves 1-to-1 with robot!
        tf2::Quaternion q_rot;
        q_rot.setRPY(roll_, pitch_, yaw_);
        tf2::Matrix3x3 R(q_rot);
        // Placed exactly between gripper claws (+0.45m forward along robot's longitudinal axis)
        tf2::Vector3 local_gripper_jaw(0.45, 0.0, -0.05);
        tf2::Vector3 world_gripper = tf2::Vector3(x_, y_, z_) + (R * local_gripper_jaw);
        ball.pose.position.x = world_gripper.x();
        ball.pose.position.y = world_gripper.y();
        ball.pose.position.z = world_gripper.z();
      } else {
        // Floating over target drum in water
        ball.pose.position.x = drum_x;
        ball.pose.position.y = drum_y;
        ball.pose.position.z = 0.70; // Floats steady above the drum opening
      }
      msg.markers.push_back(ball);
    };

    // Ball 1: Blue Flare (11.5m, 2.2m) -> Blue Drum (23.0m, 4.5m)
    renderBall(101, ball1_picked_, ball1_dropped_, ball1_drop_start_time_, 11.5, 2.2, 23.0, 4.5);
    // Ball 2: Red Flare (14.0m, 3.5m) -> Red Pinger Drum (23.0m, 1.5m)
    renderBall(102, ball2_picked_, ball2_dropped_, ball2_drop_start_time_, 14.0, 3.5, 23.0, 1.5);
    // Ball 3: Yellow Flare (13.0m, -3.5m) -> Red Drum (23.0m, -1.5m)
    renderBall(103, ball3_picked_, ball3_dropped_, ball3_drop_start_time_, 13.0, -3.5, 23.0, -1.5);

    // 3. Static labels for arena elements
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

    addText("START ZONE (140x140 cm)", 0.7, 2.0, 2.3, 1.0f, 0.9f, 0.2f);
    addText("ORANGE FLARE (Obstacle - Aligned with Gate)", 6.0, -1.0, 2.3, 1.0f, 0.4f, 0.0f);
    addText("BLUE FLARE [Ball 1]", 11.5, 2.2, 1.4, 0.2f, 0.6f, 1.0f);
    addText("RED FLARE [Ball 2]", 14.0, 3.5, 1.4, 1.0f, 0.2f, 0.2f);
    addText("YELLOW FLARE [Ball 3]", 13.0, -3.5, 1.4, 1.0f, 0.9f, 0.1f);
    addText("NAVIGATION GATE (150x100 cm)", 16.0, -1.0, 1.4, 0.2f, 1.0f, 0.3f);
    addText("BLUE DRUM [Drop 1]", 23.0, 4.5, 0.7, 0.2f, 0.5f, 1.0f);
    addText("RED PINGER DRUM [Drop 2]", 23.0, 1.5, 0.7, 1.0f, 0.2f, 0.2f);
    addText("RED DRUM [Drop 3]", 23.0, -1.5, 0.7, 1.0f, 0.3f, 0.3f);
    addText("RED DRUM 4", 23.0, -4.5, 0.7, 1.0f, 0.3f, 0.3f);

    marker_pub_->publish(msg);

    std_msgs::msg::String s_msg;
    s_msg.data = status_text_ + " | Score: " + std::to_string(score_);
    status_pub_->publish(s_msg);
  }

  void renderAndPublishCameraViews() {
    rclcpp::Time now = this->now();
    const uint32_t width = 320;
    const uint32_t height = 240;

    // 1. FRONT CAMERA FEED: 320x240 RGB8
    // Perspective looking forward (+X body axis) through underwater environment
    sensor_msgs::msg::Image front_img;
    front_img.header.stamp = now;
    front_img.header.frame_id = "front_camera_optical_frame";
    front_img.width = width;
    front_img.height = height;
    front_img.encoding = "rgb8";
    front_img.is_bigendian = false;
    front_img.step = width * 3;
    front_img.data.resize(width * height * 3);

    // Compute relative vectors to objects in front view
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    tf2::Matrix3x3 R(q);
    tf2::Matrix3x3 R_inv = R.transpose();

    // Helper to project 3D point in world to camera image plane
    auto projectFront = [&](double wx, double wy, double wz, double& u, double& v, double& depth) -> bool {
      tf2::Vector3 rel_world(wx - x_, wy - y_, wz - z_);
      tf2::Vector3 rel_cam = R_inv * rel_world;
      // Front camera optical: X_cam = -rel_cam.y(), Y_cam = -rel_cam.z(), Z_cam = rel_cam.x()
      depth = rel_cam.x();
      if (depth < 0.15 || depth > 12.0) return false;
      double fx = 240.0, fy = 240.0;
      double cx = width / 2.0, cy = height / 2.0;
      u = cx + fx * (-rel_cam.y() / depth);
      v = cy + fy * (-rel_cam.z() / depth);
      return (u >= 0 && u < width && v >= 0 && v < height);
    };

    // Helper to draw filled circle on RGB image
    auto drawDisk = [&](std::vector<uint8_t>& buf, int cu, int cv, int radius, uint8_t r, uint8_t g, uint8_t b) {
      int r2 = radius * radius;
      for (int dy = -radius; dy <= radius; ++dy) {
        int py = cv + dy;
        if (py < 0 || py >= (int)height) continue;
        for (int dx = -radius; dx <= radius; ++dx) {
          int px = cu + dx;
          if (px < 0 || px >= (int)width) continue;
          if (dx * dx + dy * dy <= r2) {
            size_t idx = (py * width + px) * 3;
            buf[idx + 0] = r;
            buf[idx + 1] = g;
            buf[idx + 2] = b;
          }
        }
      }
    };

    // Helper to draw vertical rectangle (e.g., gate post, flare)
    auto drawRect = [&](std::vector<uint8_t>& buf, int x0, int y0, int w, int h, uint8_t r, uint8_t g, uint8_t b) {
      for (int py = y0; py < y0 + h; ++py) {
        if (py < 0 || py >= (int)height) continue;
        for (int px = x0; px < x0 + w; ++px) {
          if (px < 0 || px >= (int)width) continue;
          size_t idx = (py * width + px) * 3;
          buf[idx + 0] = r;
          buf[idx + 1] = g;
          buf[idx + 2] = b;
        }
      }
    };

    // Fill Front Camera underwater background (deep blue gradient with ambient caustic light)
    for (uint32_t y = 0; y < height; ++y) {
      double ratio = (double)y / height;
      uint8_t bg_r = (uint8_t)(8 + ratio * 16);
      uint8_t bg_g = (uint8_t)(45 + ratio * 45);
      uint8_t bg_b = (uint8_t)(100 + ratio * 35);
      for (uint32_t x = 0; x < width; ++x) {
        size_t idx = (y * width + x) * 3;
        front_img.data[idx + 0] = bg_r;
        front_img.data[idx + 1] = bg_g;
        front_img.data[idx + 2] = bg_b;
      }
    }

    // Render Navigation Gate posts in front camera
    double u_g1, v_g1, d_g1;
    if (projectFront(16.0, -0.25, 0.5, u_g1, v_g1, d_g1)) {
      int gw = std::max(2, (int)(18.0 / d_g1));
      int gh = std::max(8, (int)(110.0 / d_g1));
      drawRect(front_img.data, (int)u_g1 - gw/2, (int)v_g1 - gh/2, gw, gh, 230, 30, 30); // Red Port
    }
    double u_g2, v_g2, d_g2;
    if (projectFront(16.0, -1.75, 0.5, u_g2, v_g2, d_g2)) {
      int gw = std::max(2, (int)(18.0 / d_g2));
      int gh = std::max(8, (int)(110.0 / d_g2));
      drawRect(front_img.data, (int)u_g2 - gw/2, (int)v_g2 - gh/2, gw, gh, 30, 220, 50); // Green Stbd
    }

    // Render Orange obstacle flare (aligned with gate at y = -1.0)
    double u_of, v_of, d_of;
    if (projectFront(6.0, -1.0, 1.0, u_of, v_of, d_of)) {
      int ow = std::max(3, (int)(25.0 / d_of));
      int oh = std::max(12, (int)(180.0 / d_of));
      drawRect(front_img.data, (int)u_of - ow/2, (int)v_of - oh/2, ow, oh, 255, 120, 0);
    }

    // Render Flares & Balls in Front View
    auto renderFrontProp = [&](double px, double py, double pz, uint8_t pr, uint8_t pg, uint8_t pb, bool show_ball) {
      double u, v, d;
      if (projectFront(px, py, pz, u, v, d)) {
        int pw = std::max(2, (int)(14.0 / d));
        int ph = std::max(6, (int)(80.0 / d));
        drawRect(front_img.data, (int)u - pw/2, (int)v - ph/2, pw, ph, pr, pg, pb);
        if (show_ball) {
          int br = std::max(2, (int)(16.0 / d));
          drawDisk(front_img.data, (int)u, (int)(v - ph/2 - br/2), br, 250, 250, 250); // White Ball on pole
        }
      }
    };

    renderFrontProp(11.5, 2.2, 0.4, 30, 110, 240, !ball1_picked_);   // Blue flare
    renderFrontProp(14.0, 3.5, 0.4, 230, 35, 35, !ball2_picked_);    // Red flare
    renderFrontProp(13.0, -3.5, 0.4, 245, 215, 20, !ball3_picked_);  // Yellow flare

    // Front Camera HUD Reticle (Crosshairs)
    int cx = width / 2;
    int cy = height / 2;
    for (int i = -10; i <= 10; ++i) {
      if (i != 0) {
        size_t idx_h = (cy * width + (cx + i)) * 3;
        size_t idx_v = ((cy + i) * width + cx) * 3;
        front_img.data[idx_h + 0] = 0; front_img.data[idx_h + 1] = 255; front_img.data[idx_h + 2] = 200;
        front_img.data[idx_v + 0] = 0; front_img.data[idx_v + 1] = 255; front_img.data[idx_v + 2] = 200;
      }
    }

    // 2. DOWNWARD CAMERA FEED: 320x240 RGB8
    // Perspective looking down at the pool floor (-Z body axis) to inspect drums
    sensor_msgs::msg::Image down_img;
    down_img.header.stamp = now;
    down_img.header.frame_id = "down_camera_optical_frame";
    down_img.width = width;
    down_img.height = height;
    down_img.encoding = "rgb8";
    down_img.is_bigendian = false;
    down_img.step = width * 3;
    down_img.data.resize(width * height * 3);

    // Fill Downward camera background (pool blue floor with tiles)
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        size_t idx = (y * width + x) * 3;
        bool is_tile = ((x / 32) + (y / 32)) % 2 == 0;
        uint8_t floor_b = is_tile ? 165 : 180;
        down_img.data[idx + 0] = 15;
        down_img.data[idx + 1] = 95;
        down_img.data[idx + 2] = floor_b;
      }
    }

    // Helper to project point on downward camera (looking down along -Z)
    auto projectDown = [&](double wx, double wy, double wz, double& u, double& v, double& alt) -> bool {
      tf2::Vector3 rel_world(wx - x_, wy - y_, wz - z_);
      tf2::Vector3 rel_cam = R_inv * rel_world;
      alt = -rel_cam.z();
      if (alt < 0.1 || alt > 4.0) return false;
      double fx = 180.0, fy = 180.0;
      double dcx = width / 2.0, dcy = height / 2.0;
      u = dcx + fx * (-rel_cam.y() / alt);
      v = dcy + fy * (rel_cam.x() / alt);
      return (u >= 0 && u < width && v >= 0 && v < height);
    };

    // Render Drums in downward view when flying over the drop zone
    auto renderDownDrum = [&](double dx, double dy, uint8_t r, uint8_t g, uint8_t b, bool has_ball) {
      double u, v, alt;
      if (projectDown(dx, dy, 0.15, u, v, alt)) {
        int radius = std::max(6, (int)(42.0 / alt));
        drawDisk(down_img.data, (int)u, (int)v, radius, r, g, b); // Outer rim
        drawDisk(down_img.data, (int)u, (int)v, radius - 4, (uint8_t)(r/3), (uint8_t)(g/3), (uint8_t)(b/3)); // Deep interior
        if (has_ball) {
          int br = std::max(3, (int)(15.0 / alt));
          drawDisk(down_img.data, (int)u, (int)v, br, 255, 255, 255); // Floating white ball inside drum
        }
      }
    };

    renderDownDrum(23.0, 4.5, 25, 75, 230, ball1_dropped_);   // Blue Drum
    renderDownDrum(23.0, 1.5, 220, 25, 25, ball2_dropped_);  // Red Pinger Drum
    renderDownDrum(23.0, -1.5, 220, 25, 25, ball3_dropped_); // Red Drum 2
    renderDownDrum(23.0, -4.5, 220, 25, 25, false);          // Red Drum 3

    // Downward Camera Reticle
    for (int i = -8; i <= 8; ++i) {
      if (i != 0) {
        size_t idx_h = (cy * width + (cx + i)) * 3;
        size_t idx_v = ((cy + i) * width + cx) * 3;
        down_img.data[idx_h + 0] = 255; down_img.data[idx_h + 1] = 220; down_img.data[idx_h + 2] = 0;
        down_img.data[idx_v + 0] = 255; down_img.data[idx_v + 1] = 220; down_img.data[idx_v + 2] = 0;
      }
    }

    // Publish CameraInfo messages
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = now;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.k = {240.0, 0.0, width / 2.0, 0.0, 240.0, height / 2.0, 0.0, 0.0, 1.0};

    info.header.frame_id = "front_camera_optical_frame";
    front_info_pub_->publish(info);
    front_img_pub_->publish(front_img);

    info.header.frame_id = "down_camera_optical_frame";
    down_info_pub_->publish(info);
    down_img_pub_->publish(down_img);
  }

  double x_, y_, z_;
  double roll_, pitch_, yaw_;
  double vx_ = 0.0, vy_ = 0.0, vz_ = 0.0;
  double wx_ = 0.0, wy_ = 0.0, wz_ = 0.0;

  bool is_running_ = true;
  bool autonomous_mode_ = true;
  MissionState current_state_;
  double state_timer_;
  int score_;
  std::string status_text_;

  bool ball1_picked_, ball1_dropped_;
  bool ball2_picked_, ball2_dropped_;
  bool ball3_picked_, ball3_dropped_;
  double ball1_drop_start_time_ = 0.0;
  double ball2_drop_start_time_ = 0.0;
  double ball3_drop_start_time_ = 0.0;

  double gripper_jaw_pos_ = 0.28;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr front_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr down_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr down_info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<gazebo_msgs::msg::EntityState>::SharedPtr gazebo_state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SauvcMissionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
