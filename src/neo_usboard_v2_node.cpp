/*
 * neo_usboard_v2_node.cpp
 *
 *  Created on: Oct 14, 2020
 *      Author: mad
 */

#include "rclcpp/rclcpp.hpp"

#include <vnx/vnx.h>
#include <pilot/base/CAN_Proxy.h>
#include <pilot/base/SerialPort.h>
#include <pilot/usboard/USBoardModule.h>
#include <pilot/usboard/USBoardModuleClient.hxx>
#include <neo_msgs2/msg/us_board_v2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <neo_usboard_v2/package.hxx>
#include <neo_usboard_v2/ROS_NodeBase.hxx>
#include <neo_srvs2/srv/us_board_toggle_sensor.hpp>

using std::placeholders::_1;
using std::placeholders::_2;


class ROS_Node : public rclcpp::Node, public neo_usboard_v2::ROS_NodeBase  {
public:
	ROS_Node(const std::string& _vnx_name)	:	rclcpp::Node::Node(vnx::get_process_name()),
											 neo_usboard_v2::ROS_NodeBase::ROS_NodeBase(_vnx_name) , 
									 usboard_sync("USBoardModule"){}

protected:
	void main() override
	{
		subscribe(input_data);
		subscribe(input_config);

		set_timer_millis(1000, std::bind(&ROS_Node::request_config, this));

		topicPub_usBoard = this->create_publisher<neo_msgs2::msg::USBoardV2>(topic_path + "/measurements", 1);
		srvSetChannel_active = this->create_service<neo_srvs2::srv::USBoardToggleSensor>(
			"set_channel_active", std::bind(&ROS_Node::setChannelActive, this, _1, _2));

		Super::main();

		rclcpp::shutdown();
	}

	void handle(std::shared_ptr<const pilot::usboard::USBoardData> value) override
	{
		neo_msgs2::msg::USBoardV2 usBoard;
		usBoard.header.stamp = rclcpp::Clock().now();;
		usBoard.header.frame_id = "USSensors";
		
		for(int i=0; i < 16; i++)
		{
			const bool is_active = config && config->sensor_config[i].active;
			usBoard.active[i] = is_active;

			if(is_active)
			{
				sensor_msgs::msg::Range USRangeMsg;
				USRangeMsg.header.stamp = rclcpp::Clock().now();; 		//time
				USRangeMsg.header.frame_id = "usrangesensor"+std::to_string(i);		//string
				USRangeMsg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
				USRangeMsg.field_of_view = 2.3; 		//float32 [rad]
				USRangeMsg.min_range = 0.2; 			//float32 [m]
				USRangeMsg.max_range = 3.0; 			//float32 [m]
				USRangeMsg.range = value->sensor[i]; 	//float32 [m]

				if(topicPub_USRangeSensor[i]) {
					topicPub_USRangeSensor[i]->publish(USRangeMsg);
				}
			}
			usBoard.sensor[i] = value->sensor[i];
		}

		for(int i = 0; i < 4; i++)
		{
			usBoard.analog[i] = value->analog_input[i];
		}
		topicPub_usBoard->publish(usBoard);
	}

	void handle(std::shared_ptr<const pilot::usboard::USBoardConfig> value) override
	{
		if(value->transmit_mode == pilot::usboard::USBoardConfig::TRANSMIT_MODE_REQUEST)
		{
			if (request_timer) {
				request_timer->stop();
			}
			// enable request timer
			request_timer = set_timer_millis(update_interval_ms, std::bind(&ROS_Node::update, this));
		}
		int i = 0;
		for(const auto& sensor : value->sensor_config)
		{
			if(sensor.active) {
				sensor_group_enable[i / 4] = true;		// auto enable group for requests
				topicPub_USRangeSensor[i] = this->create_publisher<sensor_msgs::msg::Range>(topic_path + "/sensor" + std::to_string(i), 1);
			}
			i++;
		}
		config = value;
		std::cout<<"Got USBoardConfig: " << value->to_string()<<std::endl;
	}

	void update()
	{
		// request sensor data
		usboard_sync.request_data(std::vector<bool>(sensor_group_enable.begin(), sensor_group_enable.end()));

		// also request analog data if enabled
		if(config && config->enable_analog_input) {
			usboard_sync.request_analog_data();
		}
	}

	void request_config()
	{
		try {
			if(!config) {
				usboard_sync.request_config();
			}
		} catch(const std::exception& ex) {
			std::cout<<"Failed to get USBoardConfig: "<<std::endl;
		}
	}

	bool setChannelActive(
		const std::shared_ptr<neo_srvs2::srv::USBoardToggleSensor::Request> req,
		std::shared_ptr<neo_srvs2::srv::USBoardToggleSensor::Response> res)
	{
		std::vector<vnx::bool_t> sensors;
		sensors.resize(16);

		for(int i = 0; i < 16; i++){
			sensors[i] = req->state[i];
		}

		usboard_sync.set_channel_active(sensors);
		
		config=nullptr;

		res->success = true;
		return res->success;
	}

private:
	pilot::usboard::USBoardModuleClient usboard_sync;

	std::shared_ptr<const pilot::usboard::USBoardConfig> config;
	std::shared_ptr<vnx::Timer> request_timer;

	rclcpp::Publisher<neo_msgs2::msg::USBoardV2>::SharedPtr topicPub_usBoard;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr topicPub_USRangeSensor[16];
	rclcpp::Service<neo_srvs2::srv::USBoardToggleSensor>::SharedPtr srvSetChannel_active;

};


int main(int argc, char** argv)
{
	// initialize ROS
	rclcpp::init(argc, argv);

	// initialize VNX
	vnx::init("neo_usboard_v2_node", 0, nullptr);
	auto nh = std::make_shared<ROS_Node>("ROS_Node");

	std::string can_device;
	std::string serial_port;
	std::string topic_path;
	int can_id = 0;
	int can_baud_rate = 0;
	double update_rate = 0.0;

	nh->declare_parameter<std::string>("can_device", "can0");
	nh->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
	nh->declare_parameter<std::string>("topic_path", "/usboard_v2");
	nh->declare_parameter<int>("can_id", 1024);
	nh->declare_parameter<int>("can_baud_rate", 1000000);
	nh->declare_parameter<double>("update_rate", 5.0);

	nh->get_parameter("can_device", can_device);
	nh->get_parameter("serial_port", serial_port);
	nh->get_parameter("topic_path", topic_path);
	nh->get_parameter("can_id", can_id);
	nh->get_parameter("can_baud_rate", can_baud_rate);
	nh->get_parameter("update_rate", update_rate);

	if(can_device != "None")
	{
		vnx::Handle<pilot::base::CAN_Proxy> module = new pilot::base::CAN_Proxy("CAN_Proxy");
		module->device = can_device;
		module->baud_rate = can_baud_rate;
		module->input = neo_usboard_v2::can_request;
		module->output = neo_usboard_v2::can_frames;
		module.start_detached();
		std::cout<<"Started " << module.get_name() << " on " << can_device <<std::endl;
	}
	else if(!serial_port.empty())
	{
		vnx::Handle<pilot::base::SerialPort> module = new pilot::base::SerialPort("SerialPort");
		module->port = serial_port;
		module->baud_rate = 19200;
		module->raw_mode = true;
		module->input = neo_usboard_v2::serial_request;
		module->output = neo_usboard_v2::serial_data;
		module.start_detached();
		std::cout<<"Started " << module.get_name() << " on " << serial_port <<std::endl;
	}
	else {
		std::cout<<"No serial port or CAN device configured!"<<std::endl;
	}

	{
		vnx::Handle<pilot::usboard::USBoardModule> module = new pilot::usboard::USBoardModule("USBoardModule");
		module->input_can = neo_usboard_v2::can_frames;
		module->input_serial = neo_usboard_v2::serial_data;
		module->topic_can_request = neo_usboard_v2::can_request;
		module->topic_serial_request = neo_usboard_v2::serial_request;
		module->output_data = neo_usboard_v2::data;
		module->output_config = neo_usboard_v2::config;
		module->can_id = can_id;
		module.start_detached();
	}
	{
		vnx::Handle<ROS_Node> module = nh;
		module->input_data = neo_usboard_v2::data;
		module->input_config = neo_usboard_v2::config;
		module->update_interval_ms = 1000 / update_rate;
		module->topic_path = topic_path;
		module.start_detached();
	}

  	rclcpp::spin(nh);	// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}
