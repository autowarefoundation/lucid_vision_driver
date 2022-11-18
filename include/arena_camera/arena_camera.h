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

#ifndef BUILD_ARENA_CAMERA_H
#define BUILD_ARENA_CAMERA_H

#include "Arena/ArenaApi.h"
#include "arena_camera/camera_settings.h"

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <future>
#include <iostream>
#include <string>
#include <thread>

class ArenaCamera : public Arena::IImageCallback
{
public:
  explicit ArenaCamera(Arena::IDevice * device, CameraSetting & camera_setting);

  ArenaCamera(
    Arena::IDevice * device, std::string & camera_name, std::string & frame_id,
    std::string & pixel_format, uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning,
    uint32_t vertical_binning);

  ArenaCamera();

  ~ArenaCamera();

  std::thread start_stream();

  void stop_stream();

  void destroy_device(Arena::ISystem * system);

  void acquisition();

  cv::Mat convert_to_image(Arena::IImage * pImage, const std::string & frame_id);

  using ImageCallbackFunction = std::function<void(std::uint32_t, cv::Mat)>;

  ImageCallbackFunction m_signal_publish_image{};

  void set_on_image_callback(ImageCallbackFunction callback);

  void OnImage(Arena::IImage * pImage)
  {
    m_signal_publish_image(m_cam_idx, convert_to_image(pImage, m_frame_id));
  }

private:
  Arena::IDevice * m_device;

  std::string m_camera_name;

  std::string m_frame_id;

  std::string m_pixel_format;

  uint32_t m_serial_no;

  uint32_t m_fps;

  uint32_t m_cam_idx;

  uint32_t m_horizontal_binning;

  uint32_t m_vertical_binning;

  uint32_t m_reached_horizontal_binning;

  uint32_t m_reached_vertical_binning;

  std::shared_future<void> future_;

  Arena::IImage * pImage = nullptr;
};

#endif  // BUILD_ARENA_CAMERA_H
