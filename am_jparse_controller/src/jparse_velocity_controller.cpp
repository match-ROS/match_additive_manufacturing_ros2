#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd & matrix, const double tolerance)
{
  if (matrix.size() == 0) {
    return Eigen::MatrixXd(matrix.cols(), matrix.rows());
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd inverse = Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());
  for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
    if (svd.singularValues()(i) > tolerance) {
      inverse(i, i) = 1.0 / svd.singularValues()(i);
    }
  }
  return svd.matrixV() * inverse * svd.matrixU().transpose();
}

Eigen::MatrixXd composeSvd(
  const Eigen::MatrixXd & u,
  const std::vector<double> & values,
  const Eigen::MatrixXd & vt)
{
  Eigen::MatrixXd sigma = Eigen::MatrixXd::Zero(u.cols(), vt.rows());
  const auto count = std::min<Eigen::Index>(
    static_cast<Eigen::Index>(values.size()), std::min(sigma.rows(), sigma.cols()));
  for (Eigen::Index i = 0; i < count; ++i) {
    sigma(i, i) = values[static_cast<std::size_t>(i)];
  }
  return u * sigma * vt;
}

Eigen::MatrixXd computeJParseInverse(
  const Eigen::MatrixXd & jacobian,
  const double gamma,
  const double position_gain,
  const double angular_gain,
  const double tolerance,
  Eigen::VectorXd * values_out,
  double * inverse_condition_out)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::MatrixXd u = svd.matrixU();
  const Eigen::MatrixXd vt = svd.matrixV().transpose();
  const Eigen::VectorXd values = svd.singularValues();
  if (values_out != nullptr) {
    *values_out = values;
  }
  const double maximum = values.size() == 0 ? 0.0 : values.maxCoeff();
  if (maximum <= std::numeric_limits<double>::epsilon()) {
    if (inverse_condition_out != nullptr) {
      *inverse_condition_out = 0.0;
    }
    return Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());
  }
  if (inverse_condition_out != nullptr) {
    *inverse_condition_out = values.minCoeff() / maximum;
  }

  const double threshold = gamma * maximum;
  std::vector<Eigen::VectorXd> stable_columns;
  std::vector<double> stable_values;
  std::vector<Eigen::VectorXd> singular_columns;
  std::vector<double> singular_phi;
  std::vector<double> safety_values;
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    const double condition = values(i) / maximum;
    if (values(i) > threshold) {
      stable_columns.push_back(u.col(i));
      stable_values.push_back(values(i));
    } else {
      singular_columns.push_back(u.col(i));
      singular_phi.push_back(condition / gamma);
    }
    safety_values.push_back(condition > gamma ? values(i) : threshold);
  }

  Eigen::MatrixXd projection = jacobian;
  if (!stable_columns.empty()) {
    Eigen::MatrixXd u_projection(jacobian.rows(), static_cast<int>(stable_columns.size()));
    for (Eigen::Index i = 0; i < u_projection.cols(); ++i) {
      u_projection.col(i) = stable_columns[static_cast<std::size_t>(i)];
    }
    projection = composeSvd(u_projection, stable_values, vt);
  }
  const Eigen::MatrixXd safety_inverse =
    pseudoInverse(composeSvd(u, safety_values, vt), tolerance);
  Eigen::MatrixXd result =
    safety_inverse * projection * pseudoInverse(projection, tolerance);
  if (!singular_columns.empty()) {
    Eigen::MatrixXd u_singular(jacobian.rows(), static_cast<int>(singular_columns.size()));
    for (Eigen::Index i = 0; i < u_singular.cols(); ++i) {
      u_singular.col(i) = singular_columns[static_cast<std::size_t>(i)];
    }
    Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(u_singular.cols(), u_singular.cols());
    for (Eigen::Index i = 0; i < phi.rows(); ++i) {
      phi(i, i) = singular_phi[static_cast<std::size_t>(i)];
    }
    Eigen::MatrixXd gains = Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
    for (Eigen::Index i = 0; i < gains.rows(); ++i) {
      gains(i, i) = i < 3 ? position_gain : angular_gain;
    }
    result += safety_inverse * u_singular * phi * u_singular.transpose() * gains;
  }
  return result;
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d & value, const double maximum)
{
  if (maximum <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double norm = value.norm();
  return norm > maximum && norm > std::numeric_limits<double>::epsilon() ?
    value * (maximum / norm) : value;
}
}  // namespace

class AmJParseController : public rclcpp::Node
{
public:
  explicit AmJParseController(const rclcpp::NodeOptions & options)
  : Node("am_jparse_velocity_controller", options)
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "robot");
    arm_ = declare_parameter<std::string>("arm", "arm");
    base_link_ = declare_parameter<std::string>("base_link", "robot_arm_base_link");
    tip_link_ = declare_parameter<std::string>("tip_link", "robot_arm_tool0");
    robot_description_topic_ = declare_parameter<std::string>(
      "robot_description_topic", "/robot/robot_description");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/robot/joint_states");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "~/twist_cmd");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/robot/arm/forward_velocity_controller/commands");
    spray_distance_topic_ = declare_parameter<std::string>(
      "spray_distance_topic", "/spray_distance_smoothed");
    fixed_tool_offset_xyz_ = declare_parameter<std::vector<double>>(
      "fixed_tool_offset_xyz", {0.0, 0.0, 0.0});
    fixed_tool_offset_quaternion_xyzw_ = declare_parameter<std::vector<double>>(
      "fixed_tool_offset_quaternion_xyzw", {0.0, 0.0, 0.0, 1.0});
    singular_values_topic_ = declare_parameter<std::string>(
      "singular_values_topic", "/am/jparse/singular_values");
    debug_twist_topic_ = declare_parameter<std::string>(
      "debug_twist_topic", "/am/jparse/debug_twist");
    readiness_topic_ = declare_parameter<std::string>("readiness_topic", "/am/jparse_ready");
    rate_hz_ = std::max(1.0, declare_parameter<double>("rate_hz", 500.0));
    command_timeout_ = declare_parameter<double>("command_timeout", 0.12);
    joint_state_timeout_ = declare_parameter<double>("joint_state_timeout", 0.5);
    readiness_heartbeat_period_ = std::max(
      0.1, declare_parameter<double>("readiness_heartbeat_period", 1.0));
    gamma_ = std::clamp(declare_parameter<double>("gamma", 0.1), 1.0e-4, 0.999);
    singular_gain_position_ = declare_parameter<double>("singular_gain_position", 1.0);
    singular_gain_angular_ = declare_parameter<double>("singular_gain_angular", 1.0);
    pinv_tolerance_ = declare_parameter<double>("pinv_tolerance", 1.0e-6);
    max_joint_velocity_ = declare_parameter<double>("max_joint_velocity", 1.5);
    max_cartesian_linear_velocity_ =
      declare_parameter<double>("max_cartesian_linear_velocity", 0.25);
    max_cartesian_angular_velocity_ =
      declare_parameter<double>("max_cartesian_angular_velocity", 0.8);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    singular_values_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(singular_values_topic_, 10);
    debug_twist_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(debug_twist_topic_, 10);
    readiness_pub_ = create_publisher<std_msgs::msg::Bool>(
      readiness_topic_, rclcpp::QoS(1).transient_local().reliable());

    const auto description_qos = rclcpp::QoS(1).transient_local().reliable();
    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic_, description_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (!chain_ready_) {
          configureChain(message->data);
        }
      });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
          if (std::isfinite(message->position[i])) {
            joint_positions_[message->name[i]] = message->position[i];
            joint_state_times_[message->name[i]] = now();
          }
        }
      });
    spray_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      spray_distance_topic_, 10,
      [this](const std_msgs::msg::Float32::SharedPtr message) {
        if (std::isfinite(message->data)) {
          spray_distance_ = static_cast<double>(message->data);
        }
      });
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        target_twist_ << message->twist.linear.x, message->twist.linear.y, message->twist.linear.z,
          message->twist.angular.x, message->twist.angular.y, message->twist.angular.z;
        target_twist_.head<3>() = clampNorm(
          target_twist_.head<3>(), max_cartesian_linear_velocity_);
        target_twist_.tail<3>() = clampNorm(
          target_twist_.tail<3>(), max_cartesian_angular_velocity_);
        last_twist_time_ = now();
        have_twist_ = true;
      });
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { update(); });
    publishReadiness(false);
  }

private:
  void configureChain(const std::string & urdf)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf, tree)) {
      RCLCPP_ERROR(get_logger(), "Could not parse robot description");
      return;
    }
    KDL::Chain chain;
    if (!tree.getChain(base_link_, tip_link_, chain)) {
      RCLCPP_ERROR(
        get_logger(), "Could not build KDL chain from '%s' to '%s'",
        base_link_.c_str(), tip_link_.c_str());
      return;
    }
    if (fixed_tool_offset_xyz_.size() != 3 || fixed_tool_offset_quaternion_xyzw_.size() != 4) {
      RCLCPP_ERROR(get_logger(), "Tool offset requires XYZ[3] and quaternion XYZW[4]");
      return;
    }
    const double qx = fixed_tool_offset_quaternion_xyzw_[0];
    const double qy = fixed_tool_offset_quaternion_xyzw_[1];
    const double qz = fixed_tool_offset_quaternion_xyzw_[2];
    const double qw = fixed_tool_offset_quaternion_xyzw_[3];
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (norm < 1.0e-9) {
      RCLCPP_ERROR(get_logger(), "Tool offset quaternion must be non-zero");
      return;
    }
    chain.addSegment(KDL::Segment(
      "am_fixed_tool_offset", KDL::Joint(KDL::Joint::None),
      KDL::Frame(
        KDL::Rotation::Quaternion(qx / norm, qy / norm, qz / norm, qw / norm),
        KDL::Vector(
          fixed_tool_offset_xyz_[0], fixed_tool_offset_xyz_[1], fixed_tool_offset_xyz_[2]))));
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      const auto joint = chain.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::None) {
        chain_joint_names_.push_back(joint.getName());
      }
    }
    if (chain_joint_names_.empty()) {
      RCLCPP_ERROR(get_logger(), "KDL chain has no movable joints");
      return;
    }
    const auto configured_names =
      declare_parameter<std::string>("command_joint_names_csv", "");
    if (configured_names.empty()) {
      command_joint_names_ = chain_joint_names_;
    } else {
      std::stringstream stream(configured_names);
      std::string name;
      while (std::getline(stream, name, ',')) {
        const auto first = name.find_first_not_of(" \t");
        if (first != std::string::npos) {
          name.erase(0, first);
          name.erase(name.find_last_not_of(" \t") + 1);
          command_joint_names_.push_back(name);
        }
      }
    }
    for (const auto & name : command_joint_names_) {
      if (std::find(chain_joint_names_.begin(), chain_joint_names_.end(), name) ==
        chain_joint_names_.end())
      {
        RCLCPP_ERROR(get_logger(), "Command joint '%s' is not part of the KDL chain", name.c_str());
        command_joint_names_.clear();
        return;
      }
    }
    chain_ = chain;
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    chain_ready_ = true;
    RCLCPP_INFO(
      get_logger(), "Configured AM J-PARSE for %s: %s -> %s",
      arm_.c_str(), base_link_.c_str(), tip_link_.c_str());
  }

  bool jointsReady() const
  {
    for (const auto & name : chain_joint_names_) {
      const auto position = joint_positions_.find(name);
      const auto timestamp = joint_state_times_.find(name);
      if (position == joint_positions_.end() || timestamp == joint_state_times_.end() ||
        (now() - timestamp->second).seconds() > joint_state_timeout_)
      {
        return false;
      }
    }
    return true;
  }

  bool readPositions(KDL::JntArray & positions) const
  {
    positions.resize(chain_joint_names_.size());
    for (std::size_t i = 0; i < chain_joint_names_.size(); ++i) {
      const auto value = joint_positions_.find(chain_joint_names_[i]);
      if (value == joint_positions_.end()) {
        return false;
      }
      positions(static_cast<unsigned int>(i)) = value->second;
    }
    return true;
  }

  void publishReadiness(const bool ready)
  {
    const auto current_time = now();
    if (
      ready == ready_ && readiness_published_ &&
      (current_time - last_readiness_publish_time_).seconds() < readiness_heartbeat_period_)
    {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = ready;
    readiness_pub_->publish(message);
    ready_ = ready;
    readiness_published_ = true;
    last_readiness_publish_time_ = current_time;
  }

  void publishCommand(const std::vector<double> & velocities)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data = velocities;
    command_pub_->publish(message);
  }

  void publishZero()
  {
    publishCommand(std::vector<double>(command_joint_names_.size(), 0.0));
  }

  void update()
  {
    const bool ready = chain_ready_ && jointsReady();
    publishReadiness(ready);
    if (!ready) {
      if (chain_ready_) {
        publishZero();
      }
      return;
    }
    if (!have_twist_ || (now() - last_twist_time_).seconds() > command_timeout_) {
      publishZero();
      return;
    }
    KDL::JntArray positions;
    if (!readPositions(positions)) {
      publishZero();
      return;
    }
    KDL::Jacobian kdl_jacobian(chain_joint_names_.size());
    KDL::Frame nozzle_frame;
    if (jac_solver_->JntToJac(positions, kdl_jacobian) < 0 ||
      fk_solver_->JntToCart(positions, nozzle_frame) < 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "KDL computation failed");
      publishZero();
      return;
    }
    Eigen::MatrixXd jacobian(6, static_cast<Eigen::Index>(chain_joint_names_.size()));
    for (unsigned int column = 0; column < kdl_jacobian.columns(); ++column) {
      for (unsigned int row = 0; row < 6; ++row) {
        jacobian(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          kdl_jacobian(row, column);
      }
    }
    const KDL::Vector deposition_offset =
      nozzle_frame.M * KDL::Vector(0.0, 0.0, spray_distance_);
    const Eigen::Vector3d offset(
      deposition_offset.x(), deposition_offset.y(), deposition_offset.z());
    Eigen::Matrix3d skew;
    skew << 0.0, -offset.z(), offset.y(),
      offset.z(), 0.0, -offset.x(),
      -offset.y(), offset.x(), 0.0;
    jacobian.topRows(3) -= skew * jacobian.bottomRows(3);

    Eigen::VectorXd singular_values;
    double inverse_condition = 0.0;
    const Eigen::MatrixXd inverse = computeJParseInverse(
      jacobian, gamma_, singular_gain_position_, singular_gain_angular_,
      pinv_tolerance_, &singular_values, &inverse_condition);
    Eigen::VectorXd velocities = inverse * target_twist_;
    if (max_joint_velocity_ > 0.0 && velocities.size() > 0) {
      const double maximum = velocities.cwiseAbs().maxCoeff();
      if (maximum > max_joint_velocity_) {
        velocities *= max_joint_velocity_ / maximum;
      }
    }
    const Eigen::VectorXd achieved_twist = jacobian * velocities;

    std::map<std::string, double> by_joint;
    for (std::size_t i = 0; i < chain_joint_names_.size(); ++i) {
      by_joint[chain_joint_names_[i]] = velocities(static_cast<Eigen::Index>(i));
    }
    std::vector<double> command;
    command.reserve(command_joint_names_.size());
    for (const auto & name : command_joint_names_) {
      command.push_back(by_joint[name]);
    }
    publishCommand(command);

    std_msgs::msg::Float64MultiArray singular_message;
    singular_message.data.push_back(inverse_condition);
    for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
      singular_message.data.push_back(singular_values(i));
    }
    singular_values_pub_->publish(singular_message);
    std_msgs::msg::Float64MultiArray debug_message;
    debug_message.data.push_back(inverse_condition);
    for (Eigen::Index i = 0; i < target_twist_.size(); ++i) {
      debug_message.data.push_back(target_twist_(i));
    }
    for (Eigen::Index i = 0; i < achieved_twist.size(); ++i) {
      debug_message.data.push_back(achieved_twist(i));
    }
    debug_twist_pub_->publish(debug_message);
  }

  std::string robot_name_;
  std::string arm_;
  std::string base_link_;
  std::string tip_link_;
  std::string robot_description_topic_;
  std::string joint_states_topic_;
  std::string twist_topic_;
  std::string command_topic_;
  std::string spray_distance_topic_;
  std::string singular_values_topic_;
  std::string debug_twist_topic_;
  std::string readiness_topic_;
  double rate_hz_;
  double command_timeout_;
  double joint_state_timeout_;
  double readiness_heartbeat_period_;
  double gamma_;
  double singular_gain_position_;
  double singular_gain_angular_;
  double pinv_tolerance_;
  double max_joint_velocity_;
  double max_cartesian_linear_velocity_;
  double max_cartesian_angular_velocity_;
  std::vector<double> fixed_tool_offset_xyz_;
  std::vector<double> fixed_tool_offset_quaternion_xyzw_;
  double spray_distance_{0.0};
  bool chain_ready_{false};
  bool have_twist_{false};
  bool ready_{false};
  bool readiness_published_{false};
  KDL::Chain chain_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> command_joint_names_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::map<std::string, double> joint_positions_;
  std::map<std::string, rclcpp::Time> joint_state_times_;
  Eigen::Matrix<double, 6, 1> target_twist_{Eigen::Matrix<double, 6, 1>::Zero()};
  rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_readiness_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr singular_values_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr readiness_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr spray_distance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef JPARSE_VELOCITY_CONTROLLER_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmJParseController>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
#endif
