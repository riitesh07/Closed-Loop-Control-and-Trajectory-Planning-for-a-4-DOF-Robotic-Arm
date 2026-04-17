#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <chrono>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>

static bool fetchDescriptionsFromAnyMoveGroup(const rclcpp::Node::SharedPtr& node,
                                              std::string& out_urdf,
                                              std::string& out_srdf,
                                              std::string& out_source_node)
{
  using namespace std::chrono_literals;

  std::vector<std::string> candidates;
  candidates.push_back("/move_group");

  auto names = node->get_node_graph_interface()->get_node_names();
  for (const auto& n : names)
    if (n.find("move_group") != std::string::npos)
      candidates.push_back(n);

  std::sort(candidates.begin(), candidates.end());
  candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

  for (const auto& target : candidates)
  {
    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, target);
    if (!client->wait_for_service(2s))
      continue;

    auto params = client->get_parameters({"robot_description", "robot_description_semantic"});

    std::string urdf, srdf;
    if (params.size() >= 1 && params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      urdf = params[0].as_string();
    if (params.size() >= 2 && params[1].get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      srdf = params[1].as_string();

    if (!urdf.empty() && !srdf.empty())
    {
      out_urdf = urdf;
      out_srdf = srdf;
      out_source_node = target;
      return true;
    }
  }
  return false;
}

static bool importMoveItDescriptionsLocal(const rclcpp::Node::SharedPtr& node)
{
  std::string urdf, srdf, src;
  if (!fetchDescriptionsFromAnyMoveGroup(node, urdf, srdf, src))
  {
    RCLCPP_ERROR(node->get_logger(), "Could not fetch robot_description + robot_description_semantic from move_group.");
    return false;
  }
  node->set_parameter(rclcpp::Parameter("robot_description", urdf));
  node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf));
  RCLCPP_INFO(node->get_logger(), "Imported URDF+SRDF from %s", src.c_str());
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("capture_tcp_pose", opts);

  if (!importMoveItDescriptionsLocal(node))
  {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&]() { exec.spin(); });

  {
    const std::string ARM_GROUP = "arm4";
    moveit::planning_interface::MoveGroupInterface mg(node, ARM_GROUP);

    rclcpp::sleep_for(std::chrono::milliseconds(600));

    std::string eef = mg.getEndEffectorLink();
    if (eef.empty()) eef = "tool0";

    auto ps = mg.getCurrentPose(eef);
    auto &p = ps.pose.position;
    auto &q = ps.pose.orientation;

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", mg.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "EEF link: %s", eef.c_str());
    RCLCPP_INFO(node->get_logger(),
                "TCP Pose:\n"
                "position: [%.6f, %.6f, %.6f]\n"
                "orientation (xyzw): [%.6f, %.6f, %.6f, %.6f]",
                p.x, p.y, p.z, q.x, q.y, q.z, q.w);
  }

  exec.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}

