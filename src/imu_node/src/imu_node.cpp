#include "imu_node/comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode() : Node("imu_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ImuPublisherNode::publishImuData, this));
    comms_.setup("/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1:1.0", 115200, 10000);
    
  }

private:
  void publishImuData()
  {
    comms_.read_imu_values(comms_.accelX,comms_.accelY,comms_.accelZ,
    comms_.gyroX,comms_.gyroY,comms_.gyroZ,comms_.yaw
    comms_.pitch,comms_.roll);
     //RCLCPP_INFO(rclcpp::get_logger("imu_node"), "Serial driver is open!");
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu4";
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.linear_acceleration.x = comms_.accelX;//0.1*9.8;//val_1 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.y = comms_.accelY;//0.2*9.8;//val_2 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.z = comms_.accelZ; //0.1*9.8;
    imu_msg.angular_velocity.x =comms_.gyroX;
    imu_msg.angular_velocity.y =comms_.gyroY;
    imu_msg.angular_velocity.z =comms_.gyroZ;
    imu_msg.orientation.x=comms_.roll;
    imu_msg.orientation.y=comms_.pitch;
    imu_msg.orientation.z=comms_.yaw;
    imu_mdh.orientaion.w=1.0;
    
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