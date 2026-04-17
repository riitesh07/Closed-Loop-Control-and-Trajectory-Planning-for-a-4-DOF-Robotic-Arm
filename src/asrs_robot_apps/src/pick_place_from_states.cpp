// pick_place_from_states.cpp
//
// SRDF-named-target pick & place demo WITH VISUAL TRAIL
// - Moves through SRDF named states (configurable sequence)
// - Publishes RED marker spheres showing robot end-effector trail
// - Optional scene spawning (disabled by default - no table/bottle)
// - Controls jaws via Float64 topic
// - RE-TIGHTENS gripper at lift and preplace to prevent slipping
// - Triggered by service: /run_sequence

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

static geometry_msgs::msg::Pose makePose(double x, double y, double z,
                                         double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y; p.position.z = z;
  p.orientation.x = qx; p.orientation.y = qy; p.orientation.z = qz; p.orientation.w = qw;
  return p;
}

static geometry_msgs::msg::Pose poseMultiply(const geometry_msgs::msg::Pose& A,
                                             const geometry_msgs::msg::Pose& B)
{
  tf2::Transform tA, tB;
  tf2::fromMsg(A, tA);
  tf2::fromMsg(B, tB);
  tf2::Transform tC = tA * tB;

  geometry_msgs::msg::Pose out;
  out.position.x = tC.getOrigin().x();
  out.position.y = tC.getOrigin().y();
  out.position.z = tC.getOrigin().z();
  out.orientation = tf2::toMsg(tC.getRotation());
  return out;
}

class PickPlaceFromStatesNode : public rclcpp::Node
{
public:
  PickPlaceFromStatesNode()
  : rclcpp::Node(
      "pick_place_from_states",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    logger_ = this->get_logger();

    // -------- defaults --------
    move_group_name_ = "arm4";
    world_frame_     = "map";
    eef_link_        = "tool0";
    attach_link_     = "tool0";

    // SRDF named states (you can override via params)
    state_sequence_  = {"home","pregrasp","grasp","lift","preplace","place","home"};

    run_on_startup_  = false;
    keep_alive_      = true;
    start_delay_sec_ = 2.0;

    // Gripper Float64 interface (0.0 = CLOSE, 1.0 = OPEN)
    jaws_topic_      = "/teensy_jaw_target";
    jaws_open_       = 1.0;
    jaws_close_      = 0.0;

    jaws_burst_count_ = 4;
    jaws_burst_dt_ms_ = 60;
    jaws_settle_ms_   = 250;
    post_move_settle_ms_ = 200;

    spawn_scene_     = false;  // DISABLED by default - no table/bottle
    clear_scene_on_start_ = true;

    table_size_xyz_  = {0.50, 0.80, 0.05};
    table_pose_      = makePose(0.35, 0.0, 0.10, 0,0,0,1);

    bottle_radius_   = 0.03;
    bottle_height_   = 0.20;
    bottle_pose_world_ = makePose(0.35, 0.10, 0.225, 0,0,0,1);
    bottle_offset_tool_ = makePose(0,0,-0.10, 0.7071,0,0,0.7071);

    planning_time_sec_ = 2.0;
    vel_scale_ = 0.5;
    acc_scale_ = 0.5;

    // Trail settings
    trail_marker_size_ = 0.08;
    trail_color_r_ = 1.0;
    trail_color_g_ = 0.0;
    trail_color_b_ = 0.0;
    trail_color_a_ = 1.0;

    // -------- read parameters --------
    this->get_parameter_or<std::string>("move_group_name", move_group_name_, move_group_name_);
    this->get_parameter_or<std::string>("world_frame",     world_frame_,     world_frame_);
    this->get_parameter_or<std::string>("eef_link",        eef_link_,        eef_link_);
    this->get_parameter_or<std::string>("attach_link",     attach_link_,     attach_link_);

    this->get_parameter_or<std::vector<std::string>>("state_sequence", state_sequence_, state_sequence_);

    this->get_parameter_or<bool>("run_on_startup",   run_on_startup_,   run_on_startup_);
    this->get_parameter_or<bool>("keep_alive",       keep_alive_,       keep_alive_);
    this->get_parameter_or<double>("start_delay_sec", start_delay_sec_, start_delay_sec_);

    this->get_parameter_or<std::string>("jaws_topic", jaws_topic_, jaws_topic_);
    this->get_parameter_or<double>("jaws_open",  jaws_open_,  jaws_open_);
    this->get_parameter_or<double>("jaws_close", jaws_close_, jaws_close_);

    this->get_parameter_or<int>("jaws_burst_count", jaws_burst_count_, jaws_burst_count_);
    this->get_parameter_or<int>("jaws_burst_dt_ms", jaws_burst_dt_ms_, jaws_burst_dt_ms_);
    this->get_parameter_or<int>("jaws_settle_ms",   jaws_settle_ms_,   jaws_settle_ms_);
    this->get_parameter_or<int>("post_move_settle_ms", post_move_settle_ms_, post_move_settle_ms_);

    this->get_parameter_or<bool>("spawn_scene", spawn_scene_, spawn_scene_);
    this->get_parameter_or<bool>("clear_scene_on_start", clear_scene_on_start_, clear_scene_on_start_);

    this->get_parameter_or<std::vector<double>>("table_size_xyz", table_size_xyz_, table_size_xyz_);

    std::vector<double> tp;
    this->get_parameter_or<std::vector<double>>(
      "table_pose_xyzw", tp, std::vector<double>{
        table_pose_.position.x, table_pose_.position.y, table_pose_.position.z,
        table_pose_.orientation.x, table_pose_.orientation.y, table_pose_.orientation.z, table_pose_.orientation.w
      });
    if (tp.size() == 7)
      table_pose_ = makePose(tp[0],tp[1],tp[2],tp[3],tp[4],tp[5],tp[6]);

    this->get_parameter_or<double>("bottle_radius", bottle_radius_, bottle_radius_);
    this->get_parameter_or<double>("bottle_height", bottle_height_, bottle_height_);

    std::vector<double> bp;
    this->get_parameter_or<std::vector<double>>(
      "bottle_pose_xyzw", bp, std::vector<double>{
        bottle_pose_world_.position.x, bottle_pose_world_.position.y, bottle_pose_world_.position.z,
        bottle_pose_world_.orientation.x, bottle_pose_world_.orientation.y, bottle_pose_world_.orientation.z, bottle_pose_world_.orientation.w
      });
    if (bp.size() == 7)
      bottle_pose_world_ = makePose(bp[0],bp[1],bp[2],bp[3],bp[4],bp[5],bp[6]);

    std::vector<double> bo;
    this->get_parameter_or<std::vector<double>>("bottle_attach_offset_xyzw", bo,
        std::vector<double>{0.0, 0.0, -0.10, 0.7071, 0.0, 0.0, 0.7071});
    if (bo.size() == 7)
      bottle_offset_tool_ = makePose(bo[0], bo[1], bo[2], bo[3], bo[4], bo[5], bo[6]);
    else if (bo.size() == 3)
      bottle_offset_tool_ = makePose(bo[0], bo[1], bo[2], 0, 0, 0, 1);

    this->get_parameter_or<double>("planning_time_sec", planning_time_sec_, planning_time_sec_);
    this->get_parameter_or<double>("vel_scale", vel_scale_, vel_scale_);
    this->get_parameter_or<double>("acc_scale", acc_scale_, acc_scale_);

    this->get_parameter_or<double>("trail_marker_size", trail_marker_size_, trail_marker_size_);

    // publishers + service
    jaws_pub_ = this->create_publisher<std_msgs::msg::Float64>(jaws_topic_, 10);
    trail_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/robot_trail", 10);

    srv_run_ = this->create_service<std_srvs::srv::Trigger>(
      "run_sequence",
      std::bind(&PickPlaceFromStatesNode::onRunService, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "PickPlace node UP (with TRAIL + RE-TIGHTEN)");
    RCLCPP_INFO(logger_, "  group=%s  eef=%s", move_group_name_.c_str(), eef_link_.c_str());
    RCLCPP_INFO(logger_, "  spawn_scene=%s (bottle: %s)",
                spawn_scene_ ? "TRUE" : "FALSE",
                spawn_scene_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(logger_, "  trail_topic=/robot_trail (marker_size=%.2fcm)", trail_marker_size_ * 100.0);
    RCLCPP_INFO(logger_, "========================================");

    if (run_on_startup_)
    {
      auto delay = std::chrono::duration<double>(start_delay_sec_);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(delay),
        std::bind(&PickPlaceFromStatesNode::startOnce, this));
      RCLCPP_INFO(logger_, "Will auto-run once after %.2f sec.", start_delay_sec_);
    }
    else
    {
      RCLCPP_INFO(logger_, "Waiting for service: /%s/run_sequence", this->get_name());
    }
  }

private:
  void onRunService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (running_.exchange(true))
    {
      res->success = false;
      res->message = "Already running";
      return;
    }

    bool ok = runSequence();

    running_.store(false);
    res->success = ok;
    res->message = ok ? "Done" : "Failed";
  }

  void startOnce()
  {
    if (started_) return;
    started_ = true;

    if (running_.exchange(true)) return;

    (void)runSequence();
    running_.store(false);

    if (!keep_alive_)
      rclcpp::shutdown();
  }

  void sleepMs(int ms)
  {
    if (ms <= 0) return;
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
  }

  void pubJawsIfChanged(double v, const char* label)
  {
    if (have_last_jaw_ && std::abs(v - last_jaw_) < 1e-9)
      return;

    std_msgs::msg::Float64 m;
    m.data = v;

    for (int i = 0; i < std::max(1, jaws_burst_count_); ++i)
    {
      jaws_pub_->publish(m);
      sleepMs(jaws_burst_dt_ms_);
    }

    last_jaw_ = v;
    have_last_jaw_ = true;

    RCLCPP_INFO(logger_, "JAWS %s -> %.3f", label, v);
    sleepMs(jaws_settle_ms_);
  }

  void clearTrail()
  {
    trail_poses_.clear();

    visualization_msgs::msg::MarkerArray clear_msg;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_msg.markers.push_back(del);
    trail_pub_->publish(clear_msg);

    RCLCPP_INFO(logger_, "Trail cleared");
  }

  void publishTrail(moveit::planning_interface::MoveGroupInterface& mg)
  {
    geometry_msgs::msg::PoseStamped pose_stamped = mg.getCurrentPose(eef_link_);

    if (pose_stamped.header.frame_id.empty())
    {
      RCLCPP_WARN(logger_, "Empty pose frame_id - check eef_link='%s'", eef_link_.c_str());
      return;
    }

    trail_poses_.push_back(pose_stamped.pose);

    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < trail_poses_.size(); ++i)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = pose_stamped.header.frame_id;
      m.header.stamp = this->now();
      m.ns = "trail";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = trail_poses_[i];
      m.scale.x = m.scale.y = m.scale.z = trail_marker_size_;
      m.color.r = trail_color_r_;
      m.color.g = trail_color_g_;
      m.color.b = trail_color_b_;
      m.color.a = trail_color_a_;
      m.lifetime = rclcpp::Duration::from_seconds(0);
      markers.markers.push_back(m);
    }

    trail_pub_->publish(markers);

    RCLCPP_INFO(logger_, "Trail: %zu markers | Pose: [%.3f, %.3f, %.3f] in '%s'",
                trail_poses_.size(),
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z,
                pose_stamped.header.frame_id.c_str());
  }

  void clearScene(moveit::planning_interface::PlanningSceneInterface& psi)
  {
    if (!clear_scene_on_start_) return;

    psi.removeCollisionObjects({"table", "bottle"});
    sleepMs(150);

    moveit_msgs::msg::AttachedCollisionObject aco_rm;
    aco_rm.link_name = attach_link_;
    aco_rm.object.id = "bottle";
    aco_rm.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyAttachedCollisionObject(aco_rm);
    sleepMs(150);
  }

  void spawnScene(moveit::planning_interface::PlanningSceneInterface& psi,
                  const std::string& frame)
  {
    if (!spawn_scene_)
    {
      RCLCPP_INFO(logger_, "Scene spawning DISABLED (no table/bottle)");
      return;
    }

    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = frame;

    shape_msgs::msg::SolidPrimitive table_prim;
    table_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_prim.dimensions = { table_size_xyz_[0], table_size_xyz_[1], table_size_xyz_[2] };

    table.primitives.push_back(table_prim);
    table.primitive_poses.push_back(table_pose_);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::CollisionObject bottle;
    bottle.id = "bottle";
    bottle.header.frame_id = frame;

    shape_msgs::msg::SolidPrimitive cyl;
    cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cyl.dimensions = { bottle_height_, bottle_radius_ };

    bottle.primitives.push_back(cyl);
    bottle.primitive_poses.push_back(bottle_pose_world_);
    bottle.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi.applyCollisionObjects({table, bottle});
    sleepMs(250);

    RCLCPP_INFO(logger_, "Scene spawned in frame '%s': table + bottle", frame.c_str());
  }

  bool attachBottle(moveit::planning_interface::PlanningSceneInterface& psi)
  {
    psi.removeCollisionObjects({"bottle"});
    sleepMs(120);

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = attach_link_;
    aco.object.id = "bottle";
    aco.object.header.frame_id = attach_link_;
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive cyl;
    cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cyl.dimensions = { bottle_height_, bottle_radius_ };

    aco.object.primitives.push_back(cyl);
    aco.object.primitive_poses.push_back(bottle_offset_tool_);

    aco.touch_links = { attach_link_, eef_link_, "tool0", "link3" };

    psi.applyAttachedCollisionObject(aco);
    sleepMs(150);

    RCLCPP_INFO(logger_, "Bottle attached to '%s'", attach_link_.c_str());
    return true;
  }

  bool detachBottleAtToolPose(moveit::planning_interface::MoveGroupInterface& mg,
                              moveit::planning_interface::PlanningSceneInterface& psi,
                              const std::string& frame_fallback)
  {
    moveit_msgs::msg::AttachedCollisionObject aco_rm;
    aco_rm.link_name = attach_link_;
    aco_rm.object.id = "bottle";
    aco_rm.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyAttachedCollisionObject(aco_rm);
    sleepMs(150);

    geometry_msgs::msg::PoseStamped tool_ps = mg.getCurrentPose(eef_link_);
    std::string frame = tool_ps.header.frame_id.empty() ? frame_fallback : tool_ps.header.frame_id;
    if (frame.empty()) frame = frame_fallback;

    geometry_msgs::msg::Pose bottle_world = poseMultiply(tool_ps.pose, bottle_offset_tool_);
    bottle_world.position.z -= (bottle_height_ / 2.0);

    moveit_msgs::msg::CollisionObject bottle;
    bottle.id = "bottle";
    bottle.header.frame_id = frame;

    shape_msgs::msg::SolidPrimitive cyl;
    cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cyl.dimensions = { bottle_height_, bottle_radius_ };

    bottle.primitives.push_back(cyl);
    bottle.primitive_poses.push_back(bottle_world);
    bottle.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi.applyCollisionObjects({bottle});
    sleepMs(150);

    RCLCPP_INFO(logger_, "Bottle detached at z=%.3f", bottle_world.position.z);
    return true;
  }

  bool moveToNamed(moveit::planning_interface::MoveGroupInterface& mg, const std::string& target)
  {
    mg.setStartStateToCurrentState();

    if (!mg.setNamedTarget(target))
    {
      RCLCPP_ERROR(logger_, "Named target '%s' not found", target.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_rc = mg.plan(plan);
    if (plan_rc != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "Planning failed at '%s' (code=%d)", target.c_str(), plan_rc.val);
      return false;
    }

    auto exec_rc = mg.execute(plan);
    if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "Execute failed at '%s' (code=%d)", target.c_str(), exec_rc.val);
      return false;
    }

    sleepMs(post_move_settle_ms_);
    return true;
  }

  bool runSequence()
  {
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "Running sequence (group=%s) ...", move_group_name_.c_str());
    RCLCPP_INFO(logger_, "========================================");

    moveit::planning_interface::MoveGroupInterface mg(this->shared_from_this(), move_group_name_);
    moveit::planning_interface::PlanningSceneInterface psi;

    mg.setPlanningTime(planning_time_sec_);
    mg.setMaxVelocityScalingFactor(std::clamp(vel_scale_, 0.01, 1.0));
    mg.setMaxAccelerationScalingFactor(std::clamp(acc_scale_, 0.01, 1.0));

    if (!eef_link_.empty())
      mg.setEndEffectorLink(eef_link_);

    rclcpp::sleep_for(600ms);

    const std::string planning_frame = mg.getPlanningFrame();
    const std::string scene_frame = world_frame_.empty() ? planning_frame : world_frame_;

    RCLCPP_INFO(logger_, "Planning frame: %s", planning_frame.c_str());
    RCLCPP_INFO(logger_, "Scene frame: %s", scene_frame.c_str());
    RCLCPP_INFO(logger_, "EEF link: %s", mg.getEndEffectorLink().c_str());

    auto targets = mg.getNamedTargets();
    std::sort(targets.begin(), targets.end());
    RCLCPP_INFO(logger_, "Named targets for '%s':", move_group_name_.c_str());
    for (const auto& t : targets) RCLCPP_INFO(logger_, "  - %s", t.c_str());

    clearScene(psi);
    spawnScene(psi, scene_frame);

    clearTrail();

    // Start with gripper open
    pubJawsIfChanged(jaws_open_, "OPEN");

    for (const auto& sname : state_sequence_)
    {
      RCLCPP_INFO(logger_, "======== STATE: %s ========", sname.c_str());

      if (!moveToNamed(mg, sname))
        return false;

      publishTrail(mg);

      if (sname == "grasp")
      {
        // Close gripper at grasp
        pubJawsIfChanged(jaws_close_, "CLOSE");
        
        sleepMs(150);
        
        // Re-close to ensure firm grip
        pubJawsIfChanged(jaws_close_, "CLOSE");

        if (spawn_scene_)
        {
          RCLCPP_INFO(logger_, "→ Attaching bottle");
          (void)attachBottle(psi);
        }
        else
        {
          RCLCPP_INFO(logger_, "→ Bottle disabled (spawn_scene=false)");
        }
      }
      else if (sname == "lift")
      {
        // RE-TIGHTEN after load comes on during lift
        RCLCPP_INFO(logger_, "→ Re-tightening grip after lift");
        //sleepMs(200);  // Let motion settle
        for (int i = 0; i < 15; ++i)
	{
	  pubJawsIfChanged(jaws_close_, "PREPLACE-CONTINUOUS");
	  sleepMs(50);
	}
      }
      else if (sname == "preplace")
      {
        // RE-TIGHTEN before moving to place position
        RCLCPP_INFO(logger_, "→ Re-tightening grip before place");
        //sleepMs(200);  // Let motion settle
        for (int i = 0; i < 40; ++i)
	{
	  pubJawsIfChanged(jaws_close_, "PREPLACE-CONTINUOUS");
	  sleepMs(50);
	}
      }
      else if (sname == "place")
      {
        // Open at place and detach object
        pubJawsIfChanged(jaws_open_, "OPEN");

        if (spawn_scene_)
        {
          RCLCPP_INFO(logger_, "→ Detaching bottle");
          (void)detachBottleAtToolPose(mg, psi, scene_frame);
        }
        else
        {
          RCLCPP_INFO(logger_, "→ Bottle disabled (spawn_scene=false)");
        }
      }
      else
      {
        // home, pregrasp, or any other state → keep jaws open
        pubJawsIfChanged(jaws_open_, "OPEN");
      }

      sleepMs(150);
    }

    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "Sequence finished: OK");
    RCLCPP_INFO(logger_, "Trail has %zu points", trail_poses_.size());
    RCLCPP_INFO(logger_, "========================================");
    return true;
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("pick_place_from_states")};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr jaws_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trail_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_run_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<bool> running_{false};
  bool started_{false};

  bool have_last_jaw_{false};
  double last_jaw_{0.0};

  std::string move_group_name_;
  std::string world_frame_;
  std::string eef_link_;
  std::string attach_link_;
  std::vector<std::string> state_sequence_;

  bool run_on_startup_{false};
  bool keep_alive_{true};
  double start_delay_sec_{2.0};

  std::string jaws_topic_;
  double jaws_open_{1.0};
  double jaws_close_{0.0};

  int jaws_burst_count_{4};
  int jaws_burst_dt_ms_{60};
  int jaws_settle_ms_{250};
  int post_move_settle_ms_{200};

  bool spawn_scene_{false};
  bool clear_scene_on_start_{true};

  std::vector<double> table_size_xyz_;
  geometry_msgs::msg::Pose table_pose_;

  double bottle_radius_{0.03};
  double bottle_height_{0.20};
  geometry_msgs::msg::Pose bottle_pose_world_;
  geometry_msgs::msg::Pose bottle_offset_tool_;

  double planning_time_sec_{2.0};
  double vel_scale_{0.5};
  double acc_scale_{0.5};

  std::vector<geometry_msgs::msg::Pose> trail_poses_;
  double trail_marker_size_{0.08};
  double trail_color_r_{1.0};
  double trail_color_g_{0.0};
  double trail_color_b_{0.0};
  double trail_color_a_{1.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceFromStatesNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

