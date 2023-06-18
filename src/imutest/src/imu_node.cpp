#include "imutest/comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode() : Node("imu_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&ImuPublisherNode::publishImuData, this));
    comms_.setup("/dev/serial/by-path/pci-0000:03:00.3-usb-0:3:1.0", 115200, 10000);
    
  }

private:
  void publishImuData()
  {
    double accel_x,gyro_z;
    comms_.read_imu_values(accel_x,gyro_z);
     RCLCPP_INFO(rclcpp::get_logger("imu_node"), "Serial driver is open!");
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.linear_acceleration.x = accel_x;//0.1*9.8;//val_1 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.y = 0.0;//0.2*9.8;//val_2 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.z = 0.0; //0.1*9.8;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z =gyro_z;
    imu_msg.orientation.x=0.0;
    imu_msg.orientation.y=0.0;
    imu_msg.orientation.z=0.0;
    
    publisher_->publish(imu_msg);
  }

  Comms comms_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisherNode>());
  rclcpp::shutdown();
  return 0;
}