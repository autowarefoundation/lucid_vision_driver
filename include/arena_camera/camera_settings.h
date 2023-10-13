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

class CameraSetting
{
public:
  explicit CameraSetting(
    const std::string & camera_name, const std::string & frame_id, const std::string & pixel_format,
    uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning, uint32_t vertical_binning,
    const std::string & url_camera_info, bool exposure_auto, float exposure_value, bool gain_auto,
    float gain_value, float gamma_value, bool enable_rectifying, bool enable_compressing, bool use_default_device_settings,
    bool image_horizontal_flip, bool image_vertical_flip)
  : m_camera_name{camera_name},
    m_frame_id{frame_id},
    m_pixel_format{pixel_format},
    m_serial_no{serial_no},
    m_fps{fps},
    m_horizontal_binning{horizontal_binning},
    m_vertical_binning{vertical_binning},
    m_url_camera_info{url_camera_info},
    m_auto_exposure_enable{exposure_auto},
    m_auto_exposure_value{exposure_value},
    m_gain_auto_enable{gain_auto},
    m_auto_gain_value{gain_value},
    m_gamma_value{gamma_value},
    m_enable_rectifying{enable_rectifying},
    m_enable_compressing{enable_compressing},
    m_use_default_device_settings{use_default_device_settings},
    m_image_horizontal_flip{image_horizontal_flip},
    m_image_vertical_flip{image_vertical_flip}
  {
    std::cout << "Camera readed from yaml file. Camera Name:" << m_camera_name
              << " Frame id:" << m_frame_id << " Serial no:" << m_serial_no
              << " Pixel_format:" << m_pixel_format << " FPS:" << m_fps << std::endl;
  }

  std::string get_camera_name() { return m_camera_name; }
  std::string get_frame_id() { return m_frame_id; }
  std::string get_pixel_format() { return m_pixel_format; }
  uint32_t get_serial_no() { return m_serial_no; }
  uint32_t get_fps() { return m_fps; }
  uint32_t get_horizontal_binning() { return m_horizontal_binning; }
  uint32_t get_vertical_binning() { return m_vertical_binning; }
  std::string get_url_camera_info() { return m_url_camera_info; }

  bool get_enable_exposure_auto() { return m_auto_exposure_enable; }
  void set_enable_auto_exposure(bool enable_exposure_auto)
  {
    m_auto_exposure_enable = enable_exposure_auto;
  }

  float get_auto_exposure_value() { return m_auto_exposure_value; }
  void set_auto_exposure_value(float exposure_value) { m_auto_exposure_value = exposure_value; }

  float get_auto_gain_value() { return m_auto_gain_value; }
  void set_auto_gain_value(float gain_value) { m_auto_gain_value = gain_value; }

  bool get_enable_gain_auto() { return m_gain_auto_enable; }
  void set_enable_auto_gain(bool enable_gain_auto) { m_gain_auto_enable = enable_gain_auto; }

  float get_gamma_value() { return m_gamma_value; }
  void set_gamma_value(float gamma_value) { m_gamma_value = gamma_value; }

  bool get_enable_rectifying() { return m_enable_rectifying; }
  void set_enable_rectifying(bool enable_rectifying)
  {
    m_enable_rectifying = enable_rectifying;
  }

  bool get_enable_compressing() { return m_enable_compressing; }
  void set_enable_compressing(bool enable_compressing)
  {
    m_enable_compressing = enable_compressing;
  }

  bool get_use_default_device_settings() { return m_use_default_device_settings; }
  void set_use_default_device_settings(bool use_default_device_settings)
  {
    m_use_default_device_settings = use_default_device_settings;
  }
  bool get_image_horizontal_flip() { return m_image_horizontal_flip; }
  void set_image_horizontal_flip(bool image_horizontal_flip)
  {
    m_image_horizontal_flip = image_horizontal_flip;
  }
  bool get_image_vertical_flip() { return m_image_vertical_flip; }
  void set_image_vertical_flip(bool image_vertical_flip)
  {
    m_image_vertical_flip = image_vertical_flip;
  }

private:
  std::string m_url_camera_info;
  std::string m_camera_name;
  std::string m_frame_id;
  std::string m_pixel_format;
  uint32_t m_serial_no;
  uint32_t m_fps;
  uint32_t m_horizontal_binning;
  uint32_t m_vertical_binning;
  bool m_auto_exposure_enable;
  float m_auto_exposure_value;  // Only relevant if m_auto_exposure_enable=true
  bool m_gain_auto_enable;
  float m_auto_gain_value;  // Only relevant if m_gain_auto_enable=true
  float m_gamma_value;
  bool m_enable_rectifying;
  bool m_enable_compressing;
  bool m_use_default_device_settings;
  bool m_image_horizontal_flip;
  bool m_image_vertical_flip;
};

#endif  // BUILD_CAMERA_SETTINGS_H
