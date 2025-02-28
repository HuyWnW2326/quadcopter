/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>



#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>



#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_attitude_setpoint_publisher_= this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            z_current  = msg->z;
            vz_current = msg->vz;
        });     

        imu_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {

            double w = msg->q[0];
            double x = msg->q[1];
            double y = msg->q[2];
            double z = msg->q[3];

            Roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) ;
            Pitch = asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
            Yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) ;

        });

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 11) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
                offboard_setpoint_counter_++;
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            // publish_trajectory_setpoint();
            

            u = PID_z_out(z_d, z_current, 2.5);
            u_out = PID_z_in(u, vz_current, 6.4);

            fz = ((9.8 - u_out) * 2.064) / (cos(Roll) * cos(Pitch));
            fz_out = -fz/f_max;

            if(fz_out <= -1.0)   fz_out = -1.0;
            else if(fz_out >= 1.0) fz_out = 1.0;
            
            publish_vehicle_attitude_setpoint(fz_out);
            RPY_to_Quaternion(0.0, 0.0, 0.0);
            
            std::cout << "z  =" << z_current << std::endl;

            std::cout << "Roll  =" << Roll << std::endl;
            std::cout << "Pitch =" << Pitch << std::endl;
            std::cout << "Yaw   =" << Yaw << std::endl;

            std::cout << "u_out  = " << u_out  << std::endl;
            std::cout << "fz     = " << fz     << std::endl;
            std::cout << "fz_out = " << fz_out << std::endl;

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);

    }

    void arm();
    void disarm();
    float PID_z_in(float DesiredValue, float CurrentValue, float Kp);
    float PID_z_out(float DesiredValue, float CurrentValue, float Kp);
    void RPY_to_Quaternion(float Roll, float Pitch, float Yaw);

private:


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr imu_subscription_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    float z_d = -15;
    float z_current;
    float vz_current;
    float Roll, Pitch, Yaw;
    float fz; 
    float fz_out;
    double f_max = 4 * 6.65858 * pow(10,-6) * pow(1000,2);          
    // double f_max = 27.7; 

    float u;
    float u_out;
    float quaternion[4];

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_vehicle_attitude_setpoint(float thrust_z);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.actuator = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = M_PI; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_attitude_setpoint(float thrust_z)
{
    VehicleAttitudeSetpoint msg{};
    msg.thrust_body[0] = 0.0;
    msg.thrust_body[1] = 0.0;
    msg.thrust_body[2] = thrust_z;
    msg.q_d[0] = quaternion[0];
    msg.q_d[1] = quaternion[1];
    msg.q_d[2] = quaternion[2];
    msg.q_d[3] = quaternion[3];
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_attitude_setpoint_publisher_->publish(msg);
}

float OffboardControl::PID_z_in(float DesiredValue, float CurrentValue, float Kp)
{
    float err;
    float PID;

    err = DesiredValue - CurrentValue;
    PID = Kp * err;
    return PID;
}

float OffboardControl::PID_z_out(float DesiredValue, float CurrentValue, float Kp)
{
    float err;
    float PID;

    err = DesiredValue - CurrentValue;

    if(err > 1) err = 1;
    else if (err < -1)  err = -1;

    PID = Kp * err;
    return PID;
}

void OffboardControl::RPY_to_Quaternion(float Roll, float Pitch, float Yaw)
{
    float cy = cos(Yaw/2);
    float sy = sin(Yaw/2);
    float cp = cos(Pitch/2);
    float sp = sin(Pitch/2);
    float cr = cos(Roll/2);
    float sr = sin(Roll/2);

    quaternion[0] = cr * cp * cy + sr * sp * sy;
    quaternion[1] = sr * cp * cy - cr * sp * sy;
    quaternion[2] = cr * sp * cy + sr * cp * sy;
    quaternion[3] = cr * cp * sy - sr * sp * cy;
}


int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}