#include "imu_node/comms.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"


class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode() : Node("imu_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ImuPublisherNode::publishImuData, this));
    comms_.setup("/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.2:1.0", 115200, 10000);
    
  }

private: 
  void publishImuData()
  {
    comms_.read_imu_values(comms_.accX,comms_.gyroZ,comms_.quatZ,
    comms_.quatW);
     //RCLCPP_INFO(rclcpp::get_logger("imu_node"), "Serial driver is open!");
    sensor_msgs::msg::Imu imu_msg;
    time=this->get_clock()->now();

    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = time;
    imu_msg.linear_acceleration.x =comms_.accX;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;
    imu_msg.angular_velocity.x =0.0;
    imu_msg.angular_velocity.y =0.0;
    imu_msg.angular_velocity.z =comms_.gyroZ;
    imu_msg.orientation.x=0.0;//comms_.accelX;
    imu_msg.orientation.y=0.0;//comms_.accelY;
    imu_msg.orientation.z=comms_.quatZ;
    imu_msg.orientation.w=comms_.quatW;
    
    publisher_->publish(imu_msg);
  }

  Comms comms_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time time;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisherNode>());
  rclcpp::shutdown();
  return 0;
}