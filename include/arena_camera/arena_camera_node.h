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

#ifndef BUILD_SRC_ARENA_CAMERA_SRC_ARENA_CAMERA_NODE_H_
#define BUILD_SRC_ARENA_CAMERA_SRC_ARENA_CAMERA_NODE_H_

#include "arena_camera/camera_settings.h"
#include "arena_cameras_handler.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <Arena/ArenaApi.h>
#include <image_geometry/pinhole_camera_model.h>

#include <chrono>
#include <thread>

class ArenaCameraNode : public ::rclcpp::Node
{
public:
  explicit ArenaCameraNode(rclcpp::NodeOptions node_options);

  CameraSetting read_camera_settings();

private:
  void publish_image(std::uint32_t camera_index, const cv::Mat & image);

  void init_camera_info(std::string camera_name, std::string camera_info_url);

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  static std::string create_camera_topic_name(std::string camera_name)
  {
    return "/lucid_vision/" + camera_name;
  }

  std::unique_ptr<ArenaCamerasHandler> m_arena_camera_handler;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  using PublisherT = ::rclcpp::Publisher<::sensor_msgs::msg::Image>;
  using CameraInfoPublisherT = ::rclcpp::Publisher<::sensor_msgs::msg::CameraInfo>;

  PublisherT::SharedPtr m_publisher{};
  PublisherT::SharedPtr m_rect_publisher{};
  rclcpp::Publisher<::sensor_msgs::msg::CompressedImage>::SharedPtr m_compressed_publisher{};
  CameraInfoPublisherT::SharedPtr m_camera_info_publisher{};

  std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info{};
  image_geometry::PinholeCameraModel m_camera_model;
  std::string m_frame_id;
};

#endif  // BUILD_SRC_ARENA_CAMERA_SRC_ARENA_CAMERA_NODE_H_
