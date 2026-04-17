#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

static const char* GROUP_NAME = "arm";
static const char* JOINTS[]   = {"continuous","revolute1","revolute2","revolute3"};
static const size_t NJ        = 4;

class Gateway : public rclcpp::Node
{
public:
  Gateway()
  : rclcpp::Node(
      "web_moveit_gateway_cpp",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)   // accept params injected from launch
        .automatically_declare_parameters_from_overrides(true))
  {
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/web/status", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/web/plan_execute_pose", rclcpp::QoS(10),
      std::bind(&Gateway::poseCb, this, std::placeholders::_1));

    joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/web/plan_execute_joint", rclcpp::QoS(10),
      std::bind(&Gateway::jointCb, this, std::placeholders::_1));

    // Try to initialize MoveIt once our node actually has the parameters.
    init_timer_ = this->create_wall_timer(500ms, [this]() {
      if (move_group_) return;

      std::string urdf, srdf;
      // Read ONLY our own node params (injected by the launch file)
      if (!this->get_parameter("robot_description", urdf) || urdf.empty() ||
          !this->get_parameter("robot_description_semantic", srdf) || srdf.empty())
      {
        static int attempt = 0;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Waiting for robot_description params (attempt %d)...", ++attempt);
        return;
      }

      try {
        // Construct after params exist; MoveGroupInterface pulls model via our parameters
        auto self = shared_from_this();
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(self, GROUP_NAME);

        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPlanningTime(2.0);

        ok("Gateway ready. Topics: /web/plan_execute_pose, /web/plan_execute_joint");
        init_timer_->cancel();
      } catch (const std::exception& e) {
        err(std::string("MoveGroup init failed: ") + e.what());
      }
    });
  }

private:
  void ok(const std::string& s){
    std_msgs::msg::String m; m.data = s;
    status_pub_->publish(m);
    RCLCPP_INFO(this->get_logger(), "%s", s.c_str());
  }
  void err(const std::string& s){
    std_msgs::msg::String m; m.data = "ERROR: " + s;
    status_pub_->publish(m);
    RCLCPP_ERROR(this->get_logger(), "%s", s.c_str());
  }
  bool ready() const {
    if (!move_group_) {
      RCLCPP_WARN(this->get_logger(), "Move group not ready yet.");
      return false;
    }
    return true;
  }

  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!ready()) return;
    try {
      move_group_->setStartStateToCurrentState();
      const std::string eef = move_group_->getEndEffectorLink();
      move_group_->setPoseTarget(msg->pose, eef);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool planned = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group_->clearPoseTargets();

      if (!planned || plan.trajectory_.joint_trajectory.points.empty()) {
        err("Pose plan failed.");
        return;
      }
      ok("Pose plan OK. Executing...");
      auto exec_ok = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (exec_ok) ok("Pose execute done."); else err("Pose execute failed.");
    } catch (const std::exception& e) {
      err(std::string("Pose exception: ") + e.what());
    }
  }

  void jointCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!ready()) return;
    try {
      if (msg->data.size() != NJ) {
        err("Expected 4 joints, got " + std::to_string(msg->data.size()));
        return;
      }

      std::map<std::string, double> name2val;
      for (size_t i = 0; i < NJ; ++i) name2val[JOINTS[i]] = msg->data[i];

      std::vector<std::string> active = move_group_->getActiveJoints();
      std::vector<double> target; target.reserve(active.size());
      for (const auto& n : active) {
        auto it = name2val.find(n);
        target.push_back(it != name2val.end() ? it->second : 0.0);
      }

      move_group_->setStartStateToCurrentState();
      move_group_->setJointValueTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool planned = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!planned || plan.trajectory_.joint_trajectory.points.empty()) {
        err("Joint plan failed.");
        return;
      }

      ok("Joint plan OK. Executing...");
      auto exec_ok = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (exec_ok) ok("Joint execute done."); else err("Joint execute failed.");
    } catch (const std::exception& e) {
      err(std::string("Joint exception: ") + e.what());
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Gateway>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

