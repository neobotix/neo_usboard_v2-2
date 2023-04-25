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
using rcl_interfaces::msg::ParameterType;

class ROS_Node : public rclcpp::Node, public neo_usboard_v2::ROS_NodeBase  {
public:
	ROS_Node(const std::string& _vnx_name)	:	rclcpp::Node::Node(vnx::get_process_name()),
											 neo_usboard_v2::ROS_NodeBase::ROS_NodeBase(_vnx_name) , 
									 usboard_sync("USBoardModule"){
									 	// Parmaters setting and declaration
		this->declare_parameter<uint8_t>("hardware_version", 0);
		this->declare_parameter<int>("serial_number", 0);

		this->declare_parameter<std::string>("can_device", "None");
		this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
		this->declare_parameter<std::string>("topic_path", "/usboard_v2");
		this->declare_parameter<int>("can_id", 1024);
		this->declare_parameter<int>("can_baud_rate", 1000000);
		this->declare_parameter<double>("update_rate", 5.0);

		this->declare_parameter<std::vector<bool>>("active_sensors", active_sensors);
		this->declare_parameter<std::vector<double>>("warn_distance", warn_distance);
		this->declare_parameter<std::vector<double>>("alarm_distance", alarm_distance);

		this->declare_parameter<std::vector<bool>>("enable_transmission", enable_transmission);
		this->declare_parameter<std::vector<uint8_t>>("resolution", resolution);
		this->declare_parameter<std::vector<uint8_t>>("fire_interval_ms", fire_interval_ms);
		this->declare_parameter<std::vector<int64_t>>("sending_sensor", sending_sensor);
		this->declare_parameter<std::vector<bool>>("cross_echo_mode", cross_echo_mode);

		this->declare_parameter<double>("low_pass_gain", 0.0);
		this->declare_parameter<bool>("enable_analog_input", false);
		this->declare_parameter<bool>("enable_legacy_format", false);
		this->declare_parameter<bool>("enable_can_termination", false);
		this->declare_parameter<bool>("relay_warn_blocked_invert", false);
		this->declare_parameter<bool>("relay_alarm_blocked_invert", false);

		this->get_parameter("can_device", can_device);
		this->get_parameter("serial_port", serial_port);
		this->get_parameter("topic_path", topic_path);
		this->get_parameter("can_id", can_id);
		this->get_parameter("can_baud_rate", can_baud_rate);
		this->get_parameter("update_rate", update_rate);

		this->get_parameter("active_sensors", active_sensors);
		this->get_parameter("warn_distance", warn_distance);
		this->get_parameter("alarm_distance", alarm_distance);

		this->get_parameter("enable_transmission", enable_transmission);
		this->get_parameter("resolution", resolution);
		this->get_parameter("fire_interval_ms", fire_interval_ms);
		this->get_parameter("sending_sensor", sending_sensor);
		this->get_parameter("cross_echo_mod", cross_echo_mode);

		this->get_parameter("low_pass_gain", low_pass_gain);
		this->get_parameter("enable_analog_input", enable_analog_input);
		this->get_parameter("enable_legacy_format", enable_legacy_format);
		this->get_parameter("enable_can_termination", enable_can_termination);
		this->get_parameter("relay_warn_blocked_invert", relay_warn_blocked_invert);
		this->get_parameter("relay_alarm_blocked_invert", relay_alarm_blocked_invert);
									 }
	void set_ros_params(std::shared_ptr<const pilot::usboard::USBoardConfig> value)
	{
		// prevent our own changes from triggering lots of param updates
		ros_params_initialized = false;

		serial_number = value->serial_number;
		hardware_version = value->hardware_version;
		can_id = value->can_id;
		can_baud_rate = value->can_baudrate;
		update_rate = value->update_interval_ms;
		
		for (auto i = 0; i < active_sensors.size(); i++) {
			active_sensors[i] = value->sensor_config[i].active;
			warn_distance[i] = value->sensor_config[i].warn_distance;
			alarm_distance[i] = value->sensor_config[i].alarm_distance;
		}

		for (auto i = 0; i < enable_transmission.size(); i++) {
			enable_transmission[i] = value->group_config[i].enable_transmission;
			resolution[i] = value->group_config[i].resolution;
			fire_interval_ms[i] = value->group_config[i].fire_interval_ms;
			sending_sensor[i] = value->group_config[i].sending_sensor;
			cross_echo_mode[i] = value->group_config[i].cross_echo_mode;
		}
		
		low_pass_gain = value->low_pass_gain;
		enable_analog_input = value->enable_analog_input;
		enable_legacy_format = value->enable_legacy_format;
		enable_can_termination = value->enable_can_termination;
		relay_warn_blocked_invert = value->relay_warn_blocked_invert;
		relay_alarm_blocked_invert = value->relay_alarm_blocked_invert;

		this->set_parameter(rclcpp::Parameter("hardware_version", hardware_version));
		this->set_parameter(rclcpp::Parameter("serial_number", serial_number));

		this->set_parameter(rclcpp::Parameter("can_id", can_id));
		this->set_parameter(rclcpp::Parameter("can_baud_rate", can_baud_rate));
		this->set_parameter(rclcpp::Parameter("update_rate", update_rate));

		this->set_parameter(rclcpp::Parameter("active_sensors", active_sensors));
		this->set_parameter(rclcpp::Parameter("warn_distance", warn_distance));
		this->set_parameter(rclcpp::Parameter("alarm_distance", alarm_distance));

		this->set_parameter(rclcpp::Parameter("enable_transmission", enable_transmission));
		this->set_parameter(rclcpp::Parameter("resolution", resolution));
		this->set_parameter(rclcpp::Parameter("fire_interval_ms", fire_interval_ms));
		this->set_parameter(rclcpp::Parameter("sending_sensor", sending_sensor));
		this->set_parameter(rclcpp::Parameter("cross_echo_mode", cross_echo_mode));

		this->set_parameter(rclcpp::Parameter("low_pass_gain", low_pass_gain));
		this->set_parameter(rclcpp::Parameter("enable_analog_input", enable_analog_input));
		this->set_parameter(rclcpp::Parameter("enable_legacy_format", enable_legacy_format));
		this->set_parameter(rclcpp::Parameter("enable_can_termination", enable_can_termination));
		this->set_parameter(rclcpp::Parameter("relay_warn_blocked_invert", relay_warn_blocked_invert));
		this->set_parameter(rclcpp::Parameter("relay_alarm_blocked_invert", relay_alarm_blocked_invert));

		ros_params_initialized = true;
	}

protected:
	void main() override
	{
		subscribe(input_data);
		subscribe(input_config);

		set_timer_millis(1000, std::bind(&ROS_Node::request_config, this));

		topicPub_usBoard = this->create_publisher<neo_msgs2::msg::USBoardV2>(topic_path + "/measurements", 1);
		dyn_params_handler = this->add_on_set_parameters_callback(
			std::bind(&ROS_Node::dynamicParametersCallback, this, std::placeholders::_1));

		Super::main();

		rclcpp::shutdown();
	}

	

	void set_pilot_params(std::shared_ptr<pilot::usboard::USBoardConfig> value) const
	{
		value->can_id = can_id;
		value->can_baudrate = can_baud_rate;
		value->update_interval_ms = update_rate;
		
		for (auto i = 0; i < active_sensors.size(); i++) {
			value->sensor_config[i].active = active_sensors[i];
			value->sensor_config[i].warn_distance = warn_distance[i];
			value->sensor_config[i].alarm_distance = alarm_distance[i];
		}

		for (auto i = 0; i < enable_transmission.size(); i++) {
			value->group_config[i].enable_transmission = enable_transmission[i];
			value->group_config[i].resolution = resolution[i];
			value->group_config[i].fire_interval_ms = fire_interval_ms[i];
			value->group_config[i].sending_sensor = sending_sensor[i];
			value->group_config[i].cross_echo_mode = cross_echo_mode[i];
		}
		
		value->low_pass_gain = low_pass_gain;
		value->enable_analog_input = enable_analog_input;
		value->enable_legacy_format = enable_legacy_format;
		value->enable_can_termination = enable_can_termination;
		value->relay_warn_blocked_invert = relay_warn_blocked_invert;
		value->relay_alarm_blocked_invert = relay_alarm_blocked_invert;
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
				USRangeMsg.radiation_type = 0; 			//uint8_t8   => Enum ULTRASOUND=0; INFRARED=1
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
		// enable or disable request timer
		if(value->transmit_mode == pilot::usboard::USBoardConfig::TRANSMIT_MODE_REQUEST)
		{
			if(!request_timer){
				request_timer = set_timer_millis(update_interval_ms, std::bind(&ROS_Node::update, this));
			}
			request_timer->reset();
		}else{
			if(request_timer){
				request_timer->stop();
			}
		}

		int i = 0;
		for(const auto& sensor : value->sensor_config)
		{
			if(sensor.active) {
				sensor_group_enable[i / 4] = true;		// auto enable group for requests
				topicPub_USRangeSensor[i] = this->create_publisher<sensor_msgs::msg::Range>(topic_path + "/sensor" + std::to_string(i), 1);
			}else{
				sensor_group_enable[i / 4] = false;
				topicPub_USRangeSensor[i] = nullptr;
			}
			i++;
		}

		config = value;
		// Map the params from Pilot to ROS as soon as the values are set
		set_ros_params(value);
		std::cout<<"Got USBoardConfig: " << value->to_string()<<std::endl;
	}

	void update()
	{
		std::lock_guard<std::mutex> lock(mutex_usboard_sync);

		// request sensor data
		usboard_sync.request_data(std::vector<bool>(sensor_group_enable.begin(), sensor_group_enable.end()));

		// also request analog data if enabled
		if(config && config->enable_analog_input) {
			usboard_sync.request_analog_data();
		}
	}

	void request_config()
	{
		std::lock_guard<std::mutex> lock(mutex_usboard_sync);

		try {
			if(!config) {
				usboard_sync.request_config();
			}
		} catch(const std::exception& ex) {
			std::cout<<"Failed to get USBoardConfig: "<<std::endl;
		}
	}

	rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
	{
		rcl_interfaces::msg::SetParametersResult result;
		if(!config || !ros_params_initialized){
			result.successful = false;
			return result;
		}
		result.successful = true;

		for (auto parameter : parameters) {
			const auto & type = parameter.get_type();
			const auto & name = parameter.get_name();

			if (type == ParameterType::PARAMETER_STRING) {
				if (name == "can_device") {
					can_device = parameter.as_string();
				} else if (name =="serial_port") {
					serial_port = parameter.as_string();
				} else if (name =="topic_path") {
					topic_path = parameter.as_string();
					std::cout<<topic_path<<std::endl;
				}
			}

			if (type == ParameterType::PARAMETER_BOOL) {
				if (name == "enable_analog_input") {
					enable_analog_input = parameter.as_bool();
				} else if (name =="enable_legacy_format") {
					enable_legacy_format = parameter.as_bool();
				} else if (name =="enable_can_termination") {
					enable_can_termination = parameter.as_bool();
				} else if (name =="relay_warn_blocked_invert") {
					relay_warn_blocked_invert = parameter.as_bool();
				} else if (name =="relay_alarm_blocked_invert") {
					relay_alarm_blocked_invert = parameter.as_bool();
				}
			}

			if (type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
				if (name == "alarm_distance") {
		      if (parameter.as_double_array().size() != 16) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 16", name.c_str());
		        result.successful = false;
		        break;
		      }
					alarm_distance = parameter.as_double_array();
				} else if (name =="warn_distance") {
					if (parameter.as_double_array().size() != 16) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 16", name.c_str());
		        result.successful = false;
		        break;
		      }
					warn_distance = parameter.as_double_array();
				}
			}

			if (type == ParameterType::PARAMETER_BOOL_ARRAY) {
				if (name == "active_sensors") {
		      if (parameter.as_bool_array().size() != 16) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 16", name.c_str());
		        result.successful = false;
		        break;
		      }
					active_sensors = parameter.as_bool_array();
				} else if (name =="enable_transmission") {
					if (parameter.as_bool_array().size() != 4) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 4", name.c_str());
		        result.successful = false;
		        break;
		      }
					enable_transmission = parameter.as_bool_array();
				} else if (name =="cross_echo_mod") {
					if (parameter.as_bool_array().size() != 4) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 4", name.c_str());
		        result.successful = false;
		        break;
		      }
					cross_echo_mode = parameter.as_bool_array();
				} 
			}

			if (type == ParameterType::PARAMETER_BYTE_ARRAY) {
				if (name == "resolution") {
		      if (parameter.as_byte_array().size() != 4) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 4", name.c_str());
		        result.successful = false;
		        break;
		      }
					resolution = parameter.as_byte_array();
				} else if (name =="fire_interval_ms") {
					if (parameter.as_byte_array().size() != 4) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 4", name.c_str());
		        result.successful = false;
		        break;
		      }
					fire_interval_ms = parameter.as_byte_array();
				} else if (name == "hardware_version") {
					RCLCPP_WARN(get_logger(), "Invalid, this parameter %s is a constant variable", name.c_str());
		        result.successful = false;
		        break;
				} else if (name =="serial_number") {
					RCLCPP_WARN(get_logger(), "Invalid, this parameter %s is a constant variable", name.c_str());
		        result.successful = false;
		        break;
				}
			}

		if (type == ParameterType::PARAMETER_INTEGER_ARRAY) {
				if (name == "sending_sensor") {
		      if (parameter.as_integer_array().size() != 4) {
		        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 3", name.c_str());
		        result.successful = false;
		        break;
		      }
					sending_sensor = parameter.as_integer_array();
				}
			}

		if (type == ParameterType::PARAMETER_DOUBLE) {
				if (name == "update_rate") {
					update_rate = parameter.as_double();
				}
			}

		if (type == ParameterType::PARAMETER_INTEGER) {
				if (name == "can_id") {
					can_id = parameter.as_int();
				} else if (name =="can_baud_rate") {
					can_baud_rate = parameter.as_int();
				} else if (name =="low_pass_gain") {
					low_pass_gain = parameter.as_int();
				}
			}

		}

		if (result.successful) {
			auto new_config = vnx::clone(config);
			set_pilot_params(new_config);
			{
				std::lock_guard<std::mutex> lock(mutex_usboard_sync);
				try{
					usboard_sync.send_config(new_config);
				}catch(const std::exception &err){
					result.successful = false;
					RCLCPP_WARN(get_logger(), "Sending USBoard config failed with: %s", err.what());
				}
			}
			if(result.successful){
				publish(new_config, input_config);
			}
		}		

		return result;
	}

private:
	pilot::usboard::USBoardModuleClient usboard_sync;

	std::shared_ptr<const pilot::usboard::USBoardConfig> config;
	std::shared_ptr<vnx::Timer> request_timer;
	std::mutex mutex_usboard_sync;

	rclcpp::Publisher<neo_msgs2::msg::USBoardV2>::SharedPtr topicPub_usBoard;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr topicPub_USRangeSensor[16];
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;

public:
	std::string can_device = "None";
	std::string serial_port = "/dev/ttyUSB0";
	std::string topic_path = "/usboard_v2";

	uint8_t serial_number;
	uint8_t hardware_version;

	int can_id = 0;
	int can_baud_rate = 0;
	double update_rate = 0.0;
	bool ros_params_initialized = false;
	
	// Our new generation supports 16 sensors. Therefore the size of the vector won't go beyond 16

	std::vector<bool> active_sensors = std::vector<bool>(16, false);
	std::vector<double> warn_distance = std::vector<double>(16, 0.0f);
	std::vector<double> alarm_distance = std::vector<double>(16, 0.0f);

	// There are 4 sensor groups, following params deals with that

	std::vector<bool> enable_transmission = std::vector<bool>(4, false);
	std::vector<uint8_t> resolution = std::vector<uint8_t>(4, 0);
	std::vector<uint8_t> fire_interval_ms = std::vector<uint8_t>(4, 0);
	std::vector<int64_t> sending_sensor = std::vector<int64_t>(4, 0);
	std::vector<bool> cross_echo_mode = std::vector<bool>(4, false);

	float low_pass_gain = 0.0f;
	bool enable_analog_input = false;
	bool enable_legacy_format = false;
	bool enable_can_termination = false;
	bool relay_warn_blocked_invert = false;
	bool relay_alarm_blocked_invert = false;

};


int main(int argc, char** argv)
{
	// initialize ROS
	rclcpp::init(argc, argv);

	// initialize VNX
	vnx::init("neo_usboard_v2_node", 0, nullptr);
	auto nh = std::make_shared<ROS_Node>("ROS_Node");

	if(nh->can_device != "None")
	{
		vnx::Handle<pilot::base::CAN_Proxy> module = new pilot::base::CAN_Proxy("CAN_Proxy");
		module->device = nh->can_device;
		module->baud_rate = nh->can_baud_rate;
		module->input = neo_usboard_v2::can_request;
		module->output = neo_usboard_v2::can_frames;
		module.start_detached();
		std::cout<<"Started " << module.get_name() << " on " << nh->can_device <<std::endl;
	}
	else if(!nh->serial_port.empty())
	{
		vnx::Handle<pilot::base::SerialPort> module = new pilot::base::SerialPort("SerialPort");
		module->port = nh->serial_port;
		module->baud_rate = 19200;
		module->raw_mode = true;
		module->input = neo_usboard_v2::serial_request;
		module->output = neo_usboard_v2::serial_data;
		module.start_detached();
		std::cout<<"Started " << module.get_name() << " on " << nh->serial_port <<std::endl;
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
		module->can_id = nh->can_id;
		module.start_detached();
	}
	{
		vnx::Handle<ROS_Node> module = nh;
		module->input_data = neo_usboard_v2::data;
		module->input_config = neo_usboard_v2::config;
		module->update_interval_ms = 1000 / nh->update_rate;
		module->topic_path = nh->topic_path;
		module.start_detached();
	}

  	rclcpp::spin(nh);	// process incoming ROS messages

	vnx::close();		// trigger VNX shutdown and wait for all modules to exit

	return 0;
}
