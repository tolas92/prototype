#include "imu_node/comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode() : Node("imu_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ImuPublisherNode::publishImuData, this));
    comms_.setup("/dev/serial/by-path/platform-xhci-hcd.8.auto-usb-0:1:1.0", 115200, 10000);
    
  }
 
private:
  void publishImuData()
  {
    comms_.read_imu_values(comms_.accelX,comms_.accelY,comms_.accelZ,
    comms_.gyroX,comms_.gyroY,comms_.gyroZ,comms_.yaw,
    comms_.pitch,comms_.roll);
     //RCLCPP_INFO(rclcpp::get_logger("imu_node"), "Serial driver is open!");
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = rclcpp::Clock().now();
    double accel_x=comms_.customRound(comms_.accelX,1);
    double gyro_z=comms_.customRound(comms_.gyroZ,1);


    imu_msg.linear_acceleration.x =0.0;//accel_x;//0.01995176249;//0.1*9.8;//val_1 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.y =0.0;//comms_.accelY;//accel_x;//comms_.accelY;//comms_.accelY;//0.2*9.8;//val_2 * 9.8 / 16384.0;
    imu_msg.linear_acceleration.z =0.0;//comms_.accelZ;//comms_.accelZ ;//comms_.accelZ; //0.1*9.8;
    imu_msg.angular_velocity.x =0.0;//comms_.gyroX;
    imu_msg.angular_velocity.y =0.0;//comms_.gyroY;
    imu_msg.angular_velocity.z =gyro_z;//comms_.gyroZ;
    imu_msg.orientation.x=0.0;//comms_.roll;
    imu_msg.orientation.y=0.0;//comms_.pitch;
    imu_msg.orientation.z=0.0;//comms_.yaw;
    imu_msg.orientation.w=1.0;

     // Set orientation covariance (3x3 matrix in row-major order)
    std::array<double, 9> orientation_covariance = {
        99999.9, 0.0, 0.0,
        0.0, 99999.9, 0.0,
        0.0, 0.0, 99999.9
    };
    imu_msg.orientation_covariance = orientation_covariance;
    
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