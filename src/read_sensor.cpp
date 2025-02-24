/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
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
 * @brief Sensor Combined uORB topic listener example
 * @file sensor_combined_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Vicente Monge
 */

 #include <rclcpp/rclcpp.hpp>
 #include <px4_msgs/msg/vehicle_local_position.hpp>
 
 /**
  * @brief Sensor Combined uORB topic data callback
  */
 class LocalPositionListener : public rclcpp::Node
 {
 public:
	 explicit LocalPositionListener() : Node("vehicle_local_position_listener")
	 {
		 rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		 auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		 
		 subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
		 [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			 std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			 std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
			 std::cout << "============================="   << std::endl;
			 std::cout << "x  =" << msg->x << std::endl;
			 std::cout << "y  =" << msg->y << std::endl;
			 std::cout << "z  =" << msg->z << std::endl;

		 });
	 }
 
 private:
	 rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
 
 };
 
 int main(int argc, char *argv[])
 {
	 std::cout << "Starting vehicle_local_posiotion listener node..." << std::endl;
	 setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	 rclcpp::init(argc, argv);
	 rclcpp::spin(std::make_shared<LocalPositionListener>());
 
	 rclcpp::shutdown();
	 return 0;
 }
 