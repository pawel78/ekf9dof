/**
 * @file lsm9ds0_node.cpp
 * @brief ROS 2 node for LSM9DS0 IMU
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lsm9ds0_imu/lsm9ds0_driver.hpp"

namespace lsm9ds0_imu {

class LSM9DS0Node : public rclcpp::Node {
public:
    LSM9DS0Node() : Node("lsm9ds0_node") {
        // Declare parameters
        this->declare_parameter("i2c_bus", 7);
        this->declare_parameter("imu_addr_gyro", 0x6B);
        this->declare_parameter("imu_addr_mag", 0x1D);
        this->declare_parameter("imu_rate_hz", 200);
        this->declare_parameter("frame_id", "imu_link");
        
        // Orientation (RPY in degrees)
        this->declare_parameter("imu_orientation_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
        
        // Calibration parameters
        this->declare_parameter("gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("accel_bias", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("mag_bias", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("accel_scale", std::vector<double>{1.0, 1.0, 1.0});
        
        // Full-scale ranges
        this->declare_parameter("gyro_scale_dps", 245);
        this->declare_parameter("accel_scale_g", 2);
        this->declare_parameter("mag_scale_gauss", 2);
        
        // Get parameters
        LSM9DS0Driver::Config config;
        config.i2c_bus = this->get_parameter("i2c_bus").as_int();
        config.gyro_accel_addr = static_cast<uint8_t>(this->get_parameter("imu_addr_gyro").as_int());
        config.mag_addr = static_cast<uint8_t>(this->get_parameter("imu_addr_mag").as_int());
        config.sample_rate_hz = this->get_parameter("imu_rate_hz").as_int();
        config.gyro_scale_dps = this->get_parameter("gyro_scale_dps").as_int();
        config.accel_scale_g = this->get_parameter("accel_scale_g").as_int();
        config.mag_scale_gauss = this->get_parameter("mag_scale_gauss").as_int();
        
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        auto orientation_vec = this->get_parameter("imu_orientation_rpy_deg").as_double_array();
        config.orientation_rpy = Eigen::Vector3d(orientation_vec[0], orientation_vec[1], orientation_vec[2]);
        
        auto gyro_bias_vec = this->get_parameter("gyro_bias").as_double_array();
        config.gyro_bias = Eigen::Vector3d(gyro_bias_vec[0], gyro_bias_vec[1], gyro_bias_vec[2]);
        
        auto accel_bias_vec = this->get_parameter("accel_bias").as_double_array();
        config.accel_bias = Eigen::Vector3d(accel_bias_vec[0], accel_bias_vec[1], accel_bias_vec[2]);
        
        auto mag_bias_vec = this->get_parameter("mag_bias").as_double_array();
        config.mag_bias = Eigen::Vector3d(mag_bias_vec[0], mag_bias_vec[1], mag_bias_vec[2]);
        
        auto accel_scale_vec = this->get_parameter("accel_scale").as_double_array();
        config.accel_scale = Eigen::Vector3d(accel_scale_vec[0], accel_scale_vec[1], accel_scale_vec[2]);
        
        // Initialize driver
        driver_ = std::make_unique<LSM9DS0Driver>(config);
        if (!driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize LSM9DS0");
            throw std::runtime_error("LSM9DS0 initialization failed");
        }
        
        RCLCPP_INFO(this->get_logger(), "LSM9DS0 initialized successfully");
        
        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 100);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 100);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temp", 10);
        
        // Create self-test service
        self_test_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/imu/self_test",
            std::bind(&LSM9DS0Node::selfTestCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Create timer for publishing
        auto period = std::chrono::microseconds(1000000 / config.sample_rate_hz);
        timer_ = this->create_wall_timer(period,
            std::bind(&LSM9DS0Node::publishData, this));
        
        RCLCPP_INFO(this->get_logger(), "LSM9DS0 node started at %d Hz", config.sample_rate_hz);
    }

private:
    void publishData() {
        Eigen::Vector3d gyro, accel, mag;
        float temp;
        
        if (!driver_->readAll(gyro, accel, mag, temp)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Failed to read IMU data");
            return;
        }
        
        auto stamp = this->now();
        
        // Publish IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = frame_id_;
        
        imu_msg.angular_velocity.x = gyro.x();
        imu_msg.angular_velocity.y = gyro.y();
        imu_msg.angular_velocity.z = gyro.z();
        
        imu_msg.linear_acceleration.x = accel.x();
        imu_msg.linear_acceleration.y = accel.y();
        imu_msg.linear_acceleration.z = accel.z();
        
        // Set covariances (diagonal)
        for (int i = 0; i < 9; ++i) {
            imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.001 : 0.0;
            imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        }
        imu_msg.orientation_covariance[0] = -1;  // Orientation not provided
        
        imu_pub_->publish(imu_msg);
        
        // Publish magnetometer message
        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.header = imu_msg.header;
        mag_msg.magnetic_field.x = mag.x();
        mag_msg.magnetic_field.y = mag.y();
        mag_msg.magnetic_field.z = mag.z();
        for (int i = 0; i < 9; ++i) {
            mag_msg.magnetic_field_covariance[i] = (i % 4 == 0) ? 1e-8 : 0.0;
        }
        mag_pub_->publish(mag_msg);
        
        // Publish temperature (less frequently)
        if (++temp_counter_ >= 20) {  // Every 20th reading (~10 Hz at 200 Hz)
            auto temp_msg = sensor_msgs::msg::Temperature();
            temp_msg.header = imu_msg.header;
            temp_msg.temperature = temp;
            temp_msg.variance = 1.0;
            temp_pub_->publish(temp_msg);
            temp_counter_ = 0;
        }
    }
    
    void selfTestCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (driver_->verifyDeviceIds()) {
            response->success = true;
            response->message = "LSM9DS0 self-test passed (WHO_AM_I: 0xD4, 0x49)";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "LSM9DS0 self-test failed (WHO_AM_I mismatch)";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }
    
    std::unique_ptr<LSM9DS0Driver> driver_;
    std::string frame_id_;
    int temp_counter_ = 0;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr self_test_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace lsm9ds0_imu

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<lsm9ds0_imu::LSM9DS0Node>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lsm9ds0_node"), "Exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
