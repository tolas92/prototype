#include "imutest/comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode() : Node("imu_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ImuPublisherNode::publishImuData, this));
    comms_.setup("/dev/ttyACM0", 115200, 10000);
    
  }

private:
  void publishImuData()
  {
    int ax,ay,az;
    double yaw, pitch, roll;
    comms_.readYPRValues(ax,ay,az,yaw, pitch, roll);
     
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "IMU_FRAME";
    imu_msg.linear_acceleration.x = ax*9.8;//0.1*9.8;//val_1 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.y = ay*9.8;//0.2*9.8;//val_2 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.z = az*9.8; //0.1*9.8;
    imu_msg.angular_velocity.x = (yaw/131.0) * 0.0174;
    imu_msg.angular_velocity.y = (pitch/131.0) * 0.0174;
    imu_msg.angular_velocity.z = (roll/131.0) * 0.0174;
    imu_msg.orientation.x=(yaw/131.0) * 0.0174;
    imu_msg.orientation.y=(pitch/131.0) * 0.0174;
    imu_msg.orientation.z=(roll/131.0) * 0.0174;
    
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