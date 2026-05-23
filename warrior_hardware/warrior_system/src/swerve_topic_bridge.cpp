
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <rclcpp/executors.hpp>
#include "warrior_system/swerve_topic_bridge.hpp"

namespace
{
    /** @brief Sums the total rotation for joint states that wrap from 2*pi to -2*pi
    when rotating in the positive direction */
    void sumRotationFromMinus2PiTo2Pi(const double current_wrapped_rad, double& total_rotation)
    {
        double delta = 0;
        angles::shortest_angular_distance_with_large_limits(total_rotation, current_wrapped_rad, 2 * M_PI, -2 * M_PI, delta);

        // Add the corrected delta to the total rotation
        total_rotation += delta;
    }
}  // namespace

namespace warrior::system
{

/***************************************************************************************/
/*******                                                                         *******/
/*******                             on_init                                     *******/
/*******                                                                         *******/
/***************************************************************************************/
CallbackReturn SwerveTopicBridge::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Initial command values
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
        const auto& component = info_.joints[i];
        for (const auto& interface : component.state_interfaces)
        {
            auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name);
            // If interface name is found in the interfaces list
            if (it != standard_interfaces_.end())
            {
                auto index = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
                // Check the initial_value param is used
                if (!interface.initial_value.empty())
                {
                    joint_states_[index][i] = std::stod(interface.initial_value);
                    joint_commands_[index][i] = std::stod(interface.initial_value);
                }
            }
        }
    }

    const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
        {
            return it->second;
        }
        return default_value;
    };

    // Add random ID to prevent warnings about multiple publishers within the same node
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", info_.name });
    topic_bridge_node_ = rclcpp::Node::make_shared("_", options);

    // Swerve Publisher
    swerve_cmd_publisher_ = topic_bridge_node_->create_publisher<warrior_msgs::msg::SwerveCmd>(
        get_hardware_parameter("swerve_command_topic", "/warrior_swerve_command"), rclcpp::QoS(1));

    // Swerve Subscriber
    swerve_state_subscriber_ = topic_bridge_node_->create_subscription<warrior_msgs::msg::SwerveState>(
        get_hardware_parameter("swerve_state_topic", "/warrior_swerve_state"), rclcpp::SensorDataQoS(),
        [this](const warrior_msgs::msg::SwerveState::SharedPtr swerve_state) { 
            latest_swerve_states_[swerve_state->swerve_id] = *swerve_state;
        });

    // if the values on the `joint_states_topic` are wrapped between -2*pi and 2*pi (like they are in Isaac Sim)
    // sum the total joint rotation returned on the `joint_states_` interface
    // if (get_hardware_parameter("sum_wrapped_joint_states", "false") == "true")
    // {
    //     sum_wrapped_joint_states_ = true;
    // }

    return CallbackReturn::SUCCESS;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                export_state_interfaces (for controller)                 *******/
/*******                                                                         *******/
/***************************************************************************************/
std::vector<hardware_interface::StateInterface> SwerveTopicBridge::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Joints' state interfaces
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
        const auto& joint = info_.joints[i];
        for (const auto& interface : joint.state_interfaces)
        {
            // Add interface: if not in the standard list then use "other" interface list
            if (!getHWInterface(joint.name, interface.name, i, joint_states_, state_interfaces))
            {
                throw std::runtime_error("Interface is not found in the standard list.");
            }
        }
    }
    return state_interfaces;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                export_command_interfaces (for controller)               *******/
/*******                                                                         *******/
/***************************************************************************************/
std::vector<hardware_interface::CommandInterface> SwerveTopicBridge::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Joints' state interfaces
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
        const auto& joint = info_.joints[i];
        for (const auto& interface : joint.command_interfaces)
        {
            if (!getHWInterface(joint.name, interface.name, i, joint_commands_, command_interfaces))
            {
                throw std::runtime_error("Interface is not found in the standard list.");
            }
        }
    }
    return command_interfaces;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                                 read                                    *******/
/*******                                                                         *******/
/***************************************************************************************/
hardware_interface::return_type SwerveTopicBridge::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (rclcpp::ok())
    {
        rclcpp::spin_some(topic_bridge_node_);
    }

    for (const auto& [id, state] : latest_swerve_states_)
    {
        const auto& joints = info_.joints;
        auto it = std::find_if(joints.begin(), joints.end(),
                            [&joint_name = std::as_const(state.name)](
                                const hardware_interface::ComponentInfo& info) { return joint_name == info.name; });
        if (it != joints.end())
        {
            joint_states_[]
            auto j = static_cast<std::size_t>(std::distance(joints.begin(), it));
            if (sum_wrapped_joint_states_)
            {
                sumRotationFromMinus2PiTo2Pi(state.position, joint_states_[POSITION_INTERFACE_INDEX][j]);
            }
            else
            {
                joint_states_[POSITION_INTERFACE_INDEX][j] = latest_swerve_states_[i].position;
            }
            if (!latest_swerve_states_[i].velocity.empty())
            {
                joint_states_[VELOCITY_INTERFACE_INDEX][j] = latest_swerve_states_[i].velocity[0];
            }
            if (!latest_swerve_states_[i].effort.empty())
            {
                joint_states_[EFFORT_INTERFACE_INDEX][j] = latest_swerve_states_[i].effort[0];
            }
        }
    }

    return hardware_interface::return_type::OK;
}

template <typename HandleType>
bool SwerveTopicBridge::getHWInterface(const std::string& name, const std::string& interface_name,
                                       const size_t vector_index, std::vector<std::vector<double>>& values,
                                       std::vector<HandleType>& interfaces)
{
    auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface_name);
    if (it != standard_interfaces_.end())
    {
        auto j = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
        interfaces.emplace_back(name, *it, &values[j][vector_index]);
        return true;
    }
    return false;
}

/***************************************************************************************/
/*******                                                                         *******/
/*******                                 write                                   *******/
/*******                                                                         *******/
/***************************************************************************************/
hardware_interface::return_type SwerveTopicBridge::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // To avoid spamming SwerveTopicBridge's joint command topic we check the difference between the joint states and
  // the current joint commands, if it's smaller than a threshold we don't publish it.
  const auto diff = std::transform_reduce(
      joint_states_[POSITION_INTERFACE_INDEX].cbegin(), joint_states_[POSITION_INTERFACE_INDEX].cend(),
      joint_commands_[POSITION_INTERFACE_INDEX].cbegin(), 0.0,
      [](const auto d1, const auto d2) { return std::abs(d1) + std::abs(d2); }, std::minus<double>{});
  if (diff <= trigger_joint_command_threshold_)
  {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState joint_state;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_state.name.push_back(info_.joints[i].name);
    joint_state.header.stamp = node_->now();
    // only send commands to the interfaces that are defined for this joint
    for (const auto& interface : info_.joints[i].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position.push_back(joint_commands_[POSITION_INTERFACE_INDEX][i]);
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity.push_back(joint_commands_[VELOCITY_INTERFACE_INDEX][i]);
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort.push_back(joint_commands_[EFFORT_INTERFACE_INDEX][i]);
      }
      else
      {
        RCLCPP_WARN_ONCE(node_->get_logger(), "Joint '%s' has unsupported command interfaces found: %s.",
                         info_.joints[i].name.c_str(), interface.name.c_str());
      }
    }
  }

  for (const auto& mimic_joint : mimic_joints_)
  {
    for (const auto& interface : info_.joints[mimic_joint.mimicked_joint_index].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.position[mimic_joint.mimicked_joint_index];
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.velocity[mimic_joint.mimicked_joint_index];
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.effort[mimic_joint.mimicked_joint_index];
      }
    }
  }

  if (rclcpp::ok())
  {
    swerve_cmd_publisher_->publish(joint_state);
  }

  return hardware_interface::return_type::OK;
}
}  // namespace warrior::system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(warrior::system::SwerveTopicBridge, hardware_interface::SystemInterface)