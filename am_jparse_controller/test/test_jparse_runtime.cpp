#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#define JPARSE_VELOCITY_CONTROLLER_NO_MAIN
#include "../src/jparse_velocity_controller.cpp"

using namespace std::chrono_literals;

namespace
{
const std::vector<std::string> kJointNames = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

std::string testUrdf()
{
  const std::vector<std::string> axes = {
    "0 0 1", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 0 1"};
  std::string urdf = "<robot name=\"jparse_test\"><link name=\"base\"/>";
  std::string parent = "base";
  for (std::size_t index = 0; index < axes.size(); ++index) {
    const std::string child = "link_" + std::to_string(index + 1);
    urdf +=
      "<link name=\"" + child + "\"/>"
      "<joint name=\"" + kJointNames[index] + "\" type=\"revolute\">"
      "<parent link=\"" + parent + "\"/><child link=\"" + child + "\"/>"
      "<origin xyz=\"0.1 0 0.1\" rpy=\"0 0 0\"/>"
      "<axis xyz=\"" + axes[index] + "\"/>"
      "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"100\" velocity=\"2.0\"/>"
      "</joint>";
    parent = child;
  }
  return urdf + "</robot>";
}

void spinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

Eigen::Matrix<double, 6, 1> toEigenTwist(
  const geometry_msgs::msg::TwistStamped & twist)
{
  Eigen::Matrix<double, 6, 1> result;
  result << twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z,
    twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z;
  return result;
}

KDL::Chain makeReferenceChain(
  const std::array<double, 3> & fixed_offset_xyz,
  const std::array<double, 4> & fixed_offset_quaternion_xyzw,
  bool include_fixed_tool_offset = true)
{
  KDL::Tree tree;
  EXPECT_TRUE(kdl_parser::treeFromString(testUrdf(), tree));
  KDL::Chain chain;
  EXPECT_TRUE(tree.getChain("base", "link_6", chain));
  if (include_fixed_tool_offset) {
    const auto & q = fixed_offset_quaternion_xyzw;
    chain.addSegment(KDL::Segment(
      "reference_fixed_tool_offset", KDL::Joint(KDL::Joint::None),
      KDL::Frame(
        KDL::Rotation::Quaternion(q[0], q[1], q[2], q[3]),
        KDL::Vector(
          fixed_offset_xyz[0], fixed_offset_xyz[1], fixed_offset_xyz[2]))));
  }
  return chain;
}

Eigen::MatrixXd referenceJacobian(
  const std::array<double, 6> & positions,
  const std::array<double, 3> & fixed_offset_xyz,
  const std::array<double, 4> & fixed_offset_quaternion_xyzw,
  double spray_distance,
  bool include_fixed_tool_offset = true)
{
  const KDL::Chain chain = makeReferenceChain(
    fixed_offset_xyz, fixed_offset_quaternion_xyzw, include_fixed_tool_offset);
  KDL::JntArray q(chain.getNrOfJoints());
  for (unsigned int index = 0; index < q.rows(); ++index) {
    q(index) = positions[index];
  }

  KDL::ChainJntToJacSolver jacobian_solver(chain);
  KDL::Jacobian kdl_jacobian(chain.getNrOfJoints());
  EXPECT_EQ(jacobian_solver.JntToJac(q, kdl_jacobian), 0);
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::Frame tip_frame;
  EXPECT_EQ(fk_solver.JntToCart(q, tip_frame), 0);
  const KDL::Vector dynamic_offset =
    tip_frame.M * KDL::Vector(0.0, 0.0, spray_distance);

  Eigen::MatrixXd result(6, static_cast<Eigen::Index>(chain.getNrOfJoints()));
  for (unsigned int col = 0; col < kdl_jacobian.columns(); ++col) {
    for (unsigned int row = 0; row < 6; ++row) {
      result(row, col) = kdl_jacobian(row, col);
    }
    const KDL::Vector angular(
      kdl_jacobian(3, col), kdl_jacobian(4, col), kdl_jacobian(5, col));
    const KDL::Vector offset_velocity = angular * dynamic_offset;
    result(0, col) += offset_velocity.x();
    result(1, col) += offset_velocity.y();
    result(2, col) += offset_velocity.z();
  }
  return result;
}

class ControllerHarness
{
public:
  ControllerHarness(
    const std::string & name,
    const std::array<double, 3> & fixed_offset_xyz = {0.0, 0.0, 0.0},
    const std::array<double, 4> & fixed_offset_quaternion_xyzw = {0.0, 0.0, 0.0, 1.0})
  : name_(name)
  {
    const auto topic = [this](const std::string & suffix) {return "/" + name_ + suffix;};
    const auto options = rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("base_link", "base"),
        rclcpp::Parameter("tip_link", "link_6"),
        rclcpp::Parameter("robot_description_topic", topic("/robot_description")),
        rclcpp::Parameter("joint_states_topic", topic("/joint_states")),
        rclcpp::Parameter("twist_topic", topic("/twist")),
        rclcpp::Parameter("command_topic", topic("/commands")),
        rclcpp::Parameter("spray_distance_topic", topic("/spray_distance")),
        rclcpp::Parameter("readiness_topic", topic("/ready")),
        rclcpp::Parameter("rate_hz", 100.0),
        rclcpp::Parameter("command_timeout", 1.0),
        rclcpp::Parameter("joint_state_timeout", 0.5),
        rclcpp::Parameter("max_joint_velocity", 100.0),
        rclcpp::Parameter("max_cartesian_linear_velocity", 100.0),
        rclcpp::Parameter("max_cartesian_angular_velocity", 100.0),
        rclcpp::Parameter(
        "fixed_tool_offset_xyz",
        std::vector<double>(fixed_offset_xyz.begin(), fixed_offset_xyz.end())),
        rclcpp::Parameter(
        "fixed_tool_offset_quaternion_xyzw",
        std::vector<double>(
          fixed_offset_quaternion_xyzw.begin(), fixed_offset_quaternion_xyzw.end())),
        rclcpp::Parameter(
        "command_joint_names_csv",
        "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6"),
    });
    controller_ = std::make_shared<AmJParseController>(options);
    node_ = std::make_shared<rclcpp::Node>(name + "_harness");
    executor_.add_node(controller_);
    executor_.add_node(node_);
    description_pub_ = node_->create_publisher<std_msgs::msg::String>(
      topic("/robot_description"), rclcpp::QoS(1).transient_local().reliable());
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      topic("/joint_states"), rclcpp::SensorDataQoS());
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      topic("/twist"), rclcpp::SystemDefaultsQoS());
    spray_distance_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
      topic("/spray_distance"), rclcpp::SystemDefaultsQoS());
    command_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic("/commands"), rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray & msg) {commands_.push_back(msg.data);});
    readiness_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic("/ready"), rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool & msg) {ready_ = ready_ || msg.data;});

    std_msgs::msg::String description;
    description.data = testUrdf();
    description_pub_->publish(description);
  }

  ~ControllerHarness()
  {
    executor_.remove_node(controller_);
    executor_.remove_node(node_);
  }

  std::vector<double> command(
    const std::array<double, 6> & positions,
    const geometry_msgs::msg::TwistStamped & twist,
    double spray_distance)
  {
    commands_.clear();
    // Drain commands that were produced for the previous test vector before
    // associating a newly received command with this request.
    spinFor(executor_, 50ms);
    commands_.clear();
    int controller_cycles = 0;
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < deadline) {
      sensor_msgs::msg::JointState joints;
      joints.header.stamp = node_->now();
      joints.name = kJointNames;
      joints.position.assign(positions.begin(), positions.end());
      joint_pub_->publish(joints);
      auto stamped_twist = twist;
      stamped_twist.header.stamp = node_->now();
      twist_pub_->publish(stamped_twist);
      std_msgs::msg::Float32 distance;
      distance.data = static_cast<float>(spray_distance);
      spray_distance_pub_->publish(distance);
      spinFor(executor_, 10ms);
      ++controller_cycles;
      if (controller_cycles >= 5 && ready_ && !commands_.empty()) {
        const auto & latest = commands_.back();
        if (latest.size() == kJointNames.size() && std::any_of(
            latest.begin(), latest.end(), [](double value) {return std::abs(value) > 1.0e-9;}))
        {
          return latest;
        }
      }
    }
    ADD_FAILURE() << "J-PARSE controller did not publish a non-zero command";
    return {};
  }

private:
  std::string name_;
  bool ready_{false};
  std::vector<std::vector<double>> commands_;
  std::shared_ptr<AmJParseController> controller_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spray_distance_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr readiness_sub_;
};

geometry_msgs::msg::TwistStamped makeTwist(const std::array<double, 6> & values)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = "base";
  twist.twist.linear.x = values[0];
  twist.twist.linear.y = values[1];
  twist.twist.linear.z = values[2];
  twist.twist.angular.x = values[3];
  twist.twist.angular.y = values[4];
  twist.twist.angular.z = values[5];
  return twist;
}
}  // namespace

class JParseRuntimeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(JParseRuntimeTest, ReportsReadinessLimitsVelocityAndZerosOnTimeout)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("base_link", "base"),
    rclcpp::Parameter("tip_link", "link_6"),
    rclcpp::Parameter("robot_description_topic", "/test_robot_description"),
    rclcpp::Parameter("joint_states_topic", "/test_joint_states"),
    rclcpp::Parameter("twist_topic", "/test_twist"),
    rclcpp::Parameter("command_topic", "/test_commands"),
    rclcpp::Parameter("readiness_topic", "/test_ready"),
    rclcpp::Parameter("rate_hz", 100.0),
    rclcpp::Parameter("command_timeout", 0.1),
    rclcpp::Parameter("joint_state_timeout", 0.5),
    rclcpp::Parameter("max_joint_velocity", 0.2),
    rclcpp::Parameter(
      "command_joint_names_csv",
      "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6"),
  });
  auto controller = std::make_shared<AmJParseController>(options);
  auto harness = std::make_shared<rclcpp::Node>("jparse_runtime_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(harness);

  auto description_pub = harness->create_publisher<std_msgs::msg::String>(
    "/test_robot_description", rclcpp::QoS(1).transient_local().reliable());
  auto joint_pub = harness->create_publisher<sensor_msgs::msg::JointState>(
    "/test_joint_states", rclcpp::SensorDataQoS());
  auto twist_pub = harness->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/test_twist", rclcpp::SystemDefaultsQoS());

  std::vector<bool> readiness;
  std::vector<std::vector<double>> commands;
  auto ready_sub = harness->create_subscription<std_msgs::msg::Bool>(
    "/test_ready", rclcpp::QoS(1).transient_local().reliable(),
    [&readiness](const std_msgs::msg::Bool & msg) {readiness.push_back(msg.data);});
  auto command_sub = harness->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/test_commands", rclcpp::SystemDefaultsQoS(),
    [&commands](const std_msgs::msg::Float64MultiArray & msg) {
      commands.push_back(msg.data);
    });

  std_msgs::msg::String description;
  description.data = testUrdf();
  description_pub->publish(description);
  sensor_msgs::msg::JointState joints;
  joints.name = kJointNames;
  joints.position.assign(kJointNames.size(), 0.1);

  const auto ready_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < ready_deadline &&
    std::find(readiness.begin(), readiness.end(), true) == readiness.end())
  {
    joints.header.stamp = harness->now();
    joint_pub->publish(joints);
    spinFor(executor, 10ms);
  }
  ASSERT_NE(std::find(readiness.begin(), readiness.end(), true), readiness.end());
  spinFor(executor, 100ms);
  ASSERT_GE(
    std::count(commands.begin(), commands.end(), std::vector<double>(6, 0.0)),
    2);

  commands.clear();
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = "base";
  twist.twist.angular.z = 10.0;
  const auto command_deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < command_deadline) {
    joints.header.stamp = harness->now();
    joint_pub->publish(joints);
    twist.header.stamp = harness->now();
    twist_pub->publish(twist);
    spinFor(executor, 10ms);
  }

  bool saw_nonzero = false;
  for (const auto & command : commands) {
    ASSERT_EQ(command.size(), 6U);
    for (const double value : command) {
      EXPECT_LE(std::abs(value), 0.200001);
      saw_nonzero = saw_nonzero || std::abs(value) > 1.0e-6;
    }
  }
  EXPECT_TRUE(saw_nonzero);

  commands.clear();
  const auto timeout_deadline = std::chrono::steady_clock::now() + 350ms;
  while (std::chrono::steady_clock::now() < timeout_deadline) {
    joints.header.stamp = harness->now();
    joint_pub->publish(joints);
    spinFor(executor, 10ms);
  }
  EXPECT_GE(
    std::count(commands.begin(), commands.end(), std::vector<double>(6, 0.0)),
    2);
}

TEST_F(JParseRuntimeTest, ThrottlesRepeatedReadinessToHeartbeatPeriod)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("base_link", "base"),
    rclcpp::Parameter("tip_link", "link_6"),
    rclcpp::Parameter("robot_description_topic", "/throttle_robot_description"),
    rclcpp::Parameter("joint_states_topic", "/throttle_joint_states"),
    rclcpp::Parameter("twist_topic", "/throttle_twist"),
    rclcpp::Parameter("command_topic", "/throttle_commands"),
    rclcpp::Parameter("readiness_topic", "/throttle_ready"),
    rclcpp::Parameter("rate_hz", 100.0),
    rclcpp::Parameter("joint_state_timeout", 0.5),
    rclcpp::Parameter("readiness_heartbeat_period", 0.5),
  });
  auto controller = std::make_shared<AmJParseController>(options);
  auto harness = std::make_shared<rclcpp::Node>("jparse_readiness_throttle_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(harness);

  auto description_pub = harness->create_publisher<std_msgs::msg::String>(
    "/throttle_robot_description", rclcpp::QoS(1).transient_local().reliable());
  auto joint_pub = harness->create_publisher<sensor_msgs::msg::JointState>(
    "/throttle_joint_states", rclcpp::SensorDataQoS());
  std::vector<bool> readiness;
  auto ready_sub = harness->create_subscription<std_msgs::msg::Bool>(
    "/throttle_ready", rclcpp::QoS(1).transient_local().reliable(),
    [&readiness](const std_msgs::msg::Bool & msg) {readiness.push_back(msg.data);});

  std_msgs::msg::String description;
  description.data = testUrdf();
  description_pub->publish(description);
  const auto publish_fresh_joint_states = [&]() {
      sensor_msgs::msg::JointState joints;
      joints.header.stamp = harness->now();
      joints.name = kJointNames;
      joints.position.assign(kJointNames.size(), 0.1);
      joint_pub->publish(joints);
    };
  const auto spin_with_fresh_joints = [&](const std::chrono::milliseconds duration) {
      const auto deadline = std::chrono::steady_clock::now() + duration;
      while (std::chrono::steady_clock::now() < deadline) {
        publish_fresh_joint_states();
        spinFor(executor, 10ms);
      }
    };

  const auto ready_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < ready_deadline &&
    std::find(readiness.begin(), readiness.end(), true) == readiness.end())
  {
    publish_fresh_joint_states();
    spinFor(executor, 10ms);
  }
  ASSERT_NE(std::find(readiness.begin(), readiness.end(), true), readiness.end());

  const auto true_messages_after_transition =
    std::count(readiness.begin(), readiness.end(), true);
  spin_with_fresh_joints(200ms);
  EXPECT_EQ(
    std::count(readiness.begin(), readiness.end(), true), true_messages_after_transition);

  spin_with_fresh_joints(600ms);
  EXPECT_GE(
    std::count(readiness.begin(), readiness.end(), true), true_messages_after_transition + 1);
}

TEST_F(JParseRuntimeTest, IdentityOffsetMatchesLegacyKdlReference)
{
  ControllerHarness controller("jparse_identity_offset_test");
  const std::array<std::array<double, 6>, 4> positions = {{
    {0.20, -0.35, 0.45, -0.30, 0.25, 0.10},
    {-0.40, 0.30, -0.25, 0.50, -0.35, 0.20},
    // Almost aligned axes: deliberately close to an unfavorable configuration.
    {0.00, 0.01, -0.01, 0.01, 0.00, 0.00},
    {0.55, -0.15, 0.20, -0.45, 0.30, -0.25},
  }};
  const std::array<std::array<double, 6>, 4> twists = {{
    {0.03, 0.00, 0.00, 0.00, 0.00, 0.00},
    {0.00, -0.02, 0.01, 0.00, 0.00, 0.00},
    {0.00, 0.00, 0.00, 0.04, -0.03, 0.02},
    {0.02, -0.01, 0.03, 0.02, 0.01, -0.03},
  }};
  const std::array<double, 3> zero_xyz = {0.0, 0.0, 0.0};
  const std::array<double, 4> identity_quaternion = {0.0, 0.0, 0.0, 1.0};

  for (const auto & q : positions) {
    for (const auto & requested_twist : twists) {
      const auto twist = makeTwist(requested_twist);
      const auto command = controller.command(q, twist, 0.0);
      ASSERT_EQ(command.size(), kJointNames.size());
      const Eigen::MatrixXd legacy_jacobian = referenceJacobian(
        q, zero_xyz, identity_quaternion, 0.0, false);
      const Eigen::VectorXd expected = computeJParseInverse(
        legacy_jacobian, 0.1, 1.0, 1.0, 1.0e-6, nullptr, nullptr) * toEigenTwist(twist);
      for (std::size_t index = 0; index < command.size(); ++index) {
        EXPECT_NEAR(command[index], expected(static_cast<Eigen::Index>(index)), 1.0e-6);
      }
    }
  }
}

TEST_F(JParseRuntimeTest, ToolAndDepositionOffsetsReconstructRequestedTwist)
{
  const std::array<double, 3> fixed_offset_xyz = {0.08, -0.03, 0.12};
  const std::array<double, 4> fixed_offset_quaternion = {
    0.0, 0.0, 0.3826834324, 0.9238795325};
  ControllerHarness controller(
    "jparse_deposition_offset_test", fixed_offset_xyz, fixed_offset_quaternion);
  const std::array<double, 6> positions = {0.25, -0.35, 0.40, -0.20, 0.30, -0.15};
  const double spray_distance = 0.14;
  const std::array<std::array<double, 6>, 3> twists = {{
    {0.03, -0.01, 0.02, 0.00, 0.00, 0.00},
    {0.00, 0.00, 0.00, 0.03, -0.02, 0.04},
    {0.02, 0.01, -0.03, -0.02, 0.03, 0.01},
  }};

  for (const auto & requested_twist : twists) {
    const auto twist = makeTwist(requested_twist);
    const auto command = controller.command(positions, twist, spray_distance);
    ASSERT_EQ(command.size(), kJointNames.size());
    const Eigen::MatrixXd deposition_jacobian = referenceJacobian(
      positions, fixed_offset_xyz, fixed_offset_quaternion, spray_distance);
    const Eigen::MatrixXd tool_jacobian = referenceJacobian(
      positions, fixed_offset_xyz, fixed_offset_quaternion, 0.0);
    EXPECT_GT((deposition_jacobian - tool_jacobian).norm(), 1.0e-6);
    Eigen::VectorXd qdot(static_cast<Eigen::Index>(command.size()));
    for (std::size_t index = 0; index < command.size(); ++index) {
      qdot(static_cast<Eigen::Index>(index)) = command[index];
    }
    const Eigen::VectorXd expected_qdot = computeJParseInverse(
      deposition_jacobian, 0.1, 1.0, 1.0, 1.0e-6, nullptr, nullptr) * toEigenTwist(twist);
    for (Eigen::Index index = 0; index < qdot.size(); ++index) {
      EXPECT_NEAR(qdot(index), expected_qdot(index), 1.0e-6);
    }
    const Eigen::VectorXd reconstructed_twist = deposition_jacobian * qdot;
    const Eigen::VectorXd expected_twist = deposition_jacobian * expected_qdot;
    for (Eigen::Index index = 0; index < reconstructed_twist.size(); ++index) {
      EXPECT_NEAR(reconstructed_twist(index), expected_twist(index), 1.0e-6);
    }
  }
}

TEST_F(JParseRuntimeTest, DynamicSprayDistanceProducesFiniteBoundedCommands)
{
  const std::array<double, 3> fixed_offset_xyz = {0.06, 0.02, 0.10};
  ControllerHarness controller("jparse_dynamic_distance_test", fixed_offset_xyz);
  const std::array<double, 6> positions = {0.30, -0.25, 0.35, -0.40, 0.20, -0.10};
  const std::array<double, 4> identity_quaternion = {0.0, 0.0, 0.0, 1.0};
  const std::array<double, 5> distances = {0.00, 0.04, 0.08, 0.04, 0.00};

  for (std::size_t index = 0; index < distances.size(); ++index) {
    const double sign = index < 3 ? 1.0 : -1.0;
    const auto twist = makeTwist({
      0.02 * sign, -0.01 * sign, 0.015,
      0.01, -0.02 * sign, 0.03});
    const auto command = controller.command(positions, twist, distances[index]);
    ASSERT_EQ(command.size(), kJointNames.size());
    Eigen::VectorXd qdot(static_cast<Eigen::Index>(command.size()));
    for (std::size_t joint = 0; joint < command.size(); ++joint) {
      EXPECT_TRUE(std::isfinite(command[joint]));
      EXPECT_LE(std::abs(command[joint]), 100.000001);
      qdot(static_cast<Eigen::Index>(joint)) = command[joint];
    }
    const Eigen::MatrixXd deposition_jacobian = referenceJacobian(
      positions, fixed_offset_xyz, identity_quaternion, distances[index]);
    const Eigen::MatrixXd tool_jacobian = referenceJacobian(
      positions, fixed_offset_xyz, identity_quaternion, 0.0);
    if (distances[index] > 0.0) {
      EXPECT_GT((deposition_jacobian - tool_jacobian).norm(), 1.0e-6);
    }
    const Eigen::VectorXd expected_qdot = computeJParseInverse(
      deposition_jacobian, 0.1, 1.0, 1.0, 1.0e-6, nullptr, nullptr) * toEigenTwist(twist);
    for (Eigen::Index joint = 0; joint < qdot.size(); ++joint) {
      EXPECT_NEAR(qdot(joint), expected_qdot(joint), 1.0e-6);
    }
    const Eigen::VectorXd reconstructed_twist = deposition_jacobian * qdot;
    const Eigen::VectorXd expected_twist = deposition_jacobian * expected_qdot;
    for (Eigen::Index row = 0; row < reconstructed_twist.size(); ++row) {
      EXPECT_NEAR(reconstructed_twist(row), expected_twist(row), 1.0e-6);
    }
  }
}
