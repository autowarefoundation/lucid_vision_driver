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

#ifndef BUILD_CAMERAS_WRAPPER_H
#define BUILD_CAMERAS_WRAPPER_H

#include "Arena/ArenaApi.h"
#include "arena_camera/arena_camera.h"
#include "arena_camera/camera_settings.h"

#include <vector>

class ArenaCamerasHandler
{
public:
  explicit ArenaCamerasHandler();

  ~ArenaCamerasHandler();

  void create_camera_from_settings(CameraSetting & camera_settings);

  void set_image_callback(ArenaCamera::ImageCallbackFunction callback);

  void start_stream();

  void stop_stream();

  void set_fps(uint32_t fps);

  GenICam_3_3_LUCID::gcstring get_auto_exposure();

  void set_auto_exposure(bool auto_exposure);

  void set_exposure_value(float exposure_value);

  GenICam_3_3_LUCID::gcstring get_auto_gain();

  void set_auto_gain(bool auto_gain);

  void set_gain_value(float gain_value);

  void set_gamma_value(float gamma_value);

  void set_enable_rectifying(bool enable_rectifying);

  bool get_enable_rectifying();

  void set_enable_compressing(bool enable_compressing);

  bool get_enable_compressing();

  void set_use_default_device_settings(bool use_default_device_settings);

  bool get_use_default_device_settings();

  void set_reverse_image_y(bool image_horizontal_flip);

  void set_reverse_image_x(bool image_vertical_flip);

private:
  ArenaCamera * m_cameras;

  Arena::ISystem * m_p_system;

  Arena::IDevice * m_device;

  bool m_enable_rectifying;
  bool m_enable_compressing;
  bool m_use_default_device_settings;
};

#endif  // BUILD_CAMERAS_WRAPPER_H
