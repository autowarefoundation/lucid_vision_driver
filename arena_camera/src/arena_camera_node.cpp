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
#include <rcutils/logging_macros.h>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <utility>

#include <signal.h>

ArenaCameraNode::ArenaCameraNode(const rclcpp::NodeOptions &node_options)
    : rclcpp::Node{"arena_camera_node", node_options} {
  m_arena_camera_handler = std::make_unique<ArenaCamerasHandler>();
  auto camera_settings = read_camera_settings();
  m_arena_camera_handler->create_cameras_from_settings(camera_settings);

  m_publishers =
      create_publishers(this, camera_settings);
  m_arena_camera_handler->set_image_callback(
      std::bind(&ArenaCameraNode::publish_image,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  m_arena_camera_handler->start_stream();
}

std::vector<CameraSetting> ArenaCameraNode::read_camera_settings() {
  std::vector<CameraSetting> camera_setting;
  const std::string cameras_param_name{"camera_names"};
  const auto camera_names{
      declare_parameter(cameras_param_name, std::vector<std::string>{})};
  RCLCPP_INFO(get_logger(), "Cameras size : %d", camera_names.size());
  for (std::size_t i = 0; i < camera_names.size(); i++) {
    std::string prefix = camera_names.at(i) + ".";
    camera_setting.emplace_back(
        camera_names.at(i),
        declare_parameter(prefix + "frame_id").template get<std::string>(),
        declare_parameter(prefix + "pixel_format").template get<std::string>(),
        declare_parameter(prefix + "serial_no").template get<uint32_t>(),
        declare_parameter(prefix + "fps").template get<uint32_t>(),
        declare_parameter(prefix + "width").template get<uint32_t>(),
        declare_parameter(prefix + "height").template get<uint32_t>(),
        declare_parameter(prefix + "flip_enable").template get<bool>(),
        declare_parameter(prefix + "camera_info_url").template get<std::string>());
  }
  return camera_setting;
}

std::vector<ArenaCameraNode::ProtectedPublisher>
ArenaCameraNode::create_publishers(::rclcpp::Node *node,
                                   std::vector<CameraSetting> camera_settings) {
  if (!node) {
    throw std::runtime_error(
        "The node is not initialized. Cannot create publishers.");
  }
  const auto number_of_publishers = camera_settings.size();
  std::vector<ProtectedPublisher> publishers(number_of_publishers);
  for (auto i = 0U; i < number_of_publishers; ++i) {
    publishers[i].set_frame_id(camera_settings.at(i).get_frame_id());

    publishers[i].init_camera_info(node, camera_settings.at(i).get_camera_name(), camera_settings.at(i).get_url_camera_info());

    publishers[i].set_publisher(
        node->create_publisher<sensor_msgs::msg::Image>(
            create_camera_topic_name(i) + "/raw_image" , rclcpp::SensorDataQoS()));

    publishers[i].set_camera_info_publisher(
        node->create_publisher<sensor_msgs::msg::CameraInfo>(
                create_camera_topic_name(i) + "/camera_info" , rclcpp::SensorDataQoS()));


  }
  return publishers;
}

void ArenaCameraNode::publish_image(std::uint32_t camera_index,
                                    const cv::Mat& image) {
  const auto publisher_index = camera_index;

  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = m_publishers.at(publisher_index).get_frame_id();

  try {
    auto camera_model = m_publishers.at(publisher_index).get_camera_model();
    cv::Mat rect;
    camera_model.rectifyImage(image, rect, 1);
    cv_bridge::CvImage img_bridge =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, rect);

    (void) img_bridge;
    img_bridge.toImageMsg(img_msg);

  } catch (...) {
    throw std::runtime_error("Runtime error, publish_image.");
  }
  m_publishers.at(publisher_index).publish(img_msg);
}

void ArenaCameraNode::ProtectedPublisher::set_publisher(
    PublisherT::SharedPtr publisher) {
  m_publisher = std::move(publisher);
}

void ArenaCameraNode::ProtectedPublisher::set_camera_info_publisher(
        CameraInfoPublisherT::SharedPtr publisher) {
  m_camera_info_publisher = std::move(publisher);
}

void ArenaCameraNode::ProtectedPublisher::publish(
    sensor_msgs::msg::Image & image_msg) {
  if (m_publisher) {
    const std::lock_guard<std::mutex> lock{m_publish_mutex};
    m_publisher->publish(std::move(image_msg));

    if(m_camera_info_publisher){
      auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(m_camera_info->getCameraInfo());
      ci->header = image_msg.header;
      m_camera_info_publisher->publish(std::move(ci));
    }

  } else {
    throw std::runtime_error("Publisher is nullptr, cannot publish.");
  }
}

void ArenaCameraNode::ProtectedPublisher::init_camera_info(::rclcpp::Node *node, std::string camera_name, std::string camera_info_url){
  m_camera_info  = std::make_shared<camera_info_manager::CameraInfoManager>(node, camera_name);
  std::cout<<"camera_name :" << camera_name<<std::endl;
  std::cout<<"camera_info :" << camera_info_url<<std::endl;
  if (m_camera_info->validateURL(camera_info_url)) {
    m_camera_info->loadCameraInfo(camera_info_url);
  } else {
    RCLCPP_WARN(node->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }

}

RCLCPP_COMPONENTS_REGISTER_NODE(ArenaCameraNode)