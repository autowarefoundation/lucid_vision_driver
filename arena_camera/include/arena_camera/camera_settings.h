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




#ifndef BUILD_CAMERA_SETTINGS_H
#define BUILD_CAMERA_SETTINGS_H

#include <iostream>
#include <string>

class CameraSetting {
public:
  explicit CameraSetting(const std::string &camera_name,
                         const std::string &frame_id,
                         const std::string &pixel_format, uint32_t serial_no,
                         uint32_t fps, uint32_t width, uint32_t height,
                         bool flip_enable, const std::string &url_camera_info)
      : m_camera_name{camera_name}, m_frame_id{frame_id},
        m_pixel_format{pixel_format}, m_serial_no{serial_no}, m_fps{fps},
        m_width{width}, m_height{height}, m_flip_enable{flip_enable}, m_url_camera_info{url_camera_info} {
    std::cout << "Camera readed from yaml file. Camera Name:" << m_camera_name
              << " Frame id:" << m_frame_id << " Serial no:" << m_serial_no
              << " Pixel_format:" << m_pixel_format << " FPS:" << m_fps
              << " Flip enable:" << m_flip_enable << std::endl;
  }

  std::string get_camera_name() { return m_camera_name; }
  std::string get_frame_id() { return m_frame_id; }
  std::string get_pixel_format() { return m_pixel_format; }
  uint32_t get_serial_no() { return m_serial_no; }
  uint32_t get_fps() { return m_fps; }
  uint32_t get_width() { return m_width; }
  uint32_t get_height() { return m_height; }
  bool get_flip_enable() { return m_flip_enable; }
  std::string get_url_camera_info() { return m_url_camera_info; }

private:
  std::string m_url_camera_info;
  std::string m_camera_name;
  std::string m_frame_id;
  std::string m_pixel_format;
  uint32_t m_serial_no;
  uint32_t m_fps;
  uint32_t m_width;
  uint32_t m_height;
  bool m_flip_enable;


};

#endif // BUILD_CAMERA_SETTINGS_H
