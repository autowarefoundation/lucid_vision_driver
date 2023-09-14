/*
* Copyright 2022 LeoDrive.ai, Inc. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
 */

#include "arena_camera/arena_camera_node.h"

#include "arena_camera/camera_settings.h"

#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rcutils/logging_macros.h>

#include <chrono>
#include <utility>

ArenaCameraNode::ArenaCameraNode(rclcpp::NodeOptions node_options)
: rclcpp::Node("arena_camera_node", node_options.allow_undeclared_parameters(true))
{
  auto camera_settings = read_camera_settings();
  m_arena_camera_handler = std::make_unique<ArenaCamerasHandler>();
  m_arena_camera_handler->create_camera_from_settings(camera_settings);
  this->m_frame_id = camera_settings.get_frame_id();
  m_camera_pub_ = image_transport::create_camera_publisher(this, camera_settings.get_camera_name() + "image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  init_camera_info(camera_settings.get_camera_name(), camera_settings.get_url_camera_info());
  m_arena_camera_handler->set_image_callback(
    std::bind(&ArenaCameraNode::publish_image, this, std::placeholders::_1, std::placeholders::_2));
  m_arena_camera_handler->start_stream();
  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArenaCameraNode::parameters_callback, this, std::placeholders::_1));
}

CameraSetting ArenaCameraNode::read_camera_settings()
{
  auto fps_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::IntegerRange fps_range;
  fps_range.set__from_value(1).set__to_value(20).set__step(1);
  fps_descriptor.integer_range = {fps_range};

  auto auto_exposure_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::IntegerRange exposure_range;
  exposure_range.set__from_value(87).set__to_value(66000).set__step(1);
  auto_exposure_descriptor.integer_range = {exposure_range};

  auto gain_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  rcl_interfaces::msg::IntegerRange gain_range;
  gain_range.set__from_value(0).set__to_value(42).set__step(1);
  gain_descriptor.integer_range = {gain_range};

  CameraSetting camera_setting(
    declare_parameter<std::string>("camera_name"), declare_parameter<std::string>("frame_id"),
    declare_parameter<std::string>("pixel_format"),
    static_cast<uint32_t>(declare_parameter<int64_t>("serial_no")),
    static_cast<uint32_t>(declare_parameter<int64_t>("fps", fps_descriptor)),
    static_cast<uint32_t>(declare_parameter<int64_t>("horizontal_binning")),
    static_cast<uint32_t>(declare_parameter<int64_t>("vertical_binning")),
    declare_parameter<std::string>("camera_info_url"),
    declare_parameter<bool>("exposure_auto"),
    static_cast<float>(declare_parameter<int64_t>("exposure_target", auto_exposure_descriptor)),
    declare_parameter<bool>("gain_auto"),
    static_cast<float>(declare_parameter<int64_t>("gain_target", gain_descriptor)),
    declare_parameter<float>("gamma_target"),
    declare_parameter<bool>("use_default_device_settings"));

  return camera_setting;
}

void ArenaCameraNode::publish_image(std::uint32_t camera_index, const cv::Mat & image)
{
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = m_frame_id;

  try {
    cv_bridge::CvImage img_bridge =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
    (void)img_bridge;
    img_bridge.toImageMsg(img_msg);
  } catch (...) {
    throw std::runtime_error("Runtime error, publish_image.");
  }  
  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(m_camera_info->getCameraInfo());
  if( !((ci->width == img_msg.width) &&(ci->height == img_msg.height)))
  {
    ci->width = img_msg.width;
    ci->height = img_msg.height;
  }
  ci->header = img_msg.header;
  m_camera_pub_.publish( img_msg, *(ci));
}

void ArenaCameraNode::init_camera_info(std::string camera_name, std::string camera_info_url)
{
  m_camera_info = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name);
  if (m_camera_info->validateURL(camera_info_url)) {
    m_camera_info->loadCameraInfo(camera_info_url);
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }
}

rcl_interfaces::msg::SetParametersResult ArenaCameraNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  auto print_status = [this](const rclcpp::Parameter & parameter) {
    RCLCPP_INFO(this->get_logger(), "%s", parameter.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", parameter.value_to_string().c_str());
  };

  for (const auto & param : parameters) {
    if (param.get_name() == "fps") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (param.as_int() >= 1 && param.as_int() <= 20) {
          m_arena_camera_handler->set_fps(param.as_int());
          result.successful = true;
          print_status(param);
        }
      }
    }

    if (param.get_name() == "exposure_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_auto_exposure(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "exposure_target") {
      if (
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        m_arena_camera_handler->set_exposure_value(static_cast<float>(param.as_int()));
        result.successful = true;
        print_status(param);
      }
    }
    if (param.get_name() == "gain_auto") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_auto_gain(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "gain_target") {
      if (
        param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        m_arena_camera_handler->set_gain_value(static_cast<float>(param.as_int()));
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "gamma_target") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        m_arena_camera_handler->set_gamma_value(param.as_double());
        result.successful = true;
        print_status(param);
      }
    }

    if (param.get_name() == "use_default_device_settings") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_arena_camera_handler->set_use_default_device_settings(param.as_bool());
        result.successful = true;
        print_status(param);
      }
    }
  }

  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ArenaCameraNode)