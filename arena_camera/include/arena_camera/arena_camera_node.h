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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <Arena/ArenaApi.h>

#include <chrono>
#include <thread>

#include "arena_camera/camera_settings.h"
#include "arena_cameras_handler.h"

class ArenaCameraNode: public ::rclcpp::Node {
public:
  explicit ArenaCameraNode(const rclcpp::NodeOptions & node_options);

  std::vector < CameraSetting > read_camera_settings();

private:
  class ProtectedPublisher;

  void publish_image(
    std::uint32_t camera_index,
    const cv::Mat& image);

  static std::vector < ProtectedPublisher > create_publishers(
    ::rclcpp::Node * node,
    std::vector<CameraSetting> camera_settings);

  static std::string create_camera_topic_name(std::uint32_t camera_index)
  {
    return "/lucid_vision/camera_" + std::to_string(camera_index) ;
  }

  std::unique_ptr < ArenaCamerasHandler > m_arena_camera_handler;

  std::vector < ProtectedPublisher > m_publishers {};
};

class ArenaCameraNode::ProtectedPublisher
{
    using PublisherT = ::rclcpp::Publisher<::sensor_msgs::msg::Image>;
    using CameraInfoPublisherT = ::rclcpp::Publisher<::sensor_msgs::msg::CameraInfo>;

public:

    void set_publisher(PublisherT::SharedPtr publisher);
    void set_camera_info_publisher(CameraInfoPublisherT::SharedPtr publisher);
    void publish(sensor_msgs::msg::Image & image_msg);
    void init_camera_info(::rclcpp::Node *node, std::string camera_name, std::string camera_info_url);

    void set_frame_id(std::string frame_id){
      m_frame_id = frame_id;
    }

    std::string get_frame_id(){
      return m_frame_id;
    }

    image_geometry::PinholeCameraModel get_camera_model(){
      auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(m_camera_info->getCameraInfo());
      m_camera_model.fromCameraInfo(*ci);
      return m_camera_model;
    }


private:
    std::mutex m_publish_mutex{};
    PublisherT::SharedPtr m_publisher{};
    CameraInfoPublisherT::SharedPtr m_camera_info_publisher{};
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info{};
    image_geometry::PinholeCameraModel m_camera_model;
    std::string m_frame_id;
};


#endif //BUILD_SRC_ARENA_CAMERA_SRC_ARENA_CAMERA_NODE_H_
