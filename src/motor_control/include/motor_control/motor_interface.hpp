#ifndef MOTOR_CONTROL_MOTOR_INTERFACE_H
#define MOTOR_CONTROL_MOTOR_INTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "motor_control/config.h"
#include "motor_control/comms.h"
#include "motor_control/wheel.h"
#include "rclcpp/rclcpp.hpp"


using hardware_interface::return_type;
namespace motor_control
{
class MotorControl: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
    public : 
    RCLCPP_SHARED_PTR_DEFINITIONS(MotorControl)
    MotorControl();
    //return_type on_init(const hardware_interface::HardwareInfo &info) override;
    return_type configure(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    return_type start() override;
    return_type stop() override;
    return_type read()override;
    return_type write() override;

    private:
    Config cfg_;
    Comms arduino_;
    Wheel l_wheel_back;
    Wheel r_wheel_back;
    rclcpp::Logger log_;
    //Parameters for the robot
    double hw_start_sec_;
    double hw_stop_sec_;
    //Store the commands for the robot
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    std::chrono::time_point<std::chrono::system_clock> time_;
    
    
};
}//namespace for the robot

#endif
