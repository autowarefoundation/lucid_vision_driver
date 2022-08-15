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

class ArenaCamerasHandler {
public:
  explicit ArenaCamerasHandler();

  ~ArenaCamerasHandler();

  void init_arena();

  void
  create_cameras_from_settings(std::vector<CameraSetting> &camera_settings);

  void set_image_callback(ArenaCamera::ImageCallbackFunction callback);

  void start_stream();

  void stop_stream();

  int get_camera_count() { return m_cameras.size(); }

private:
  std::vector<ArenaCamera> m_cameras;

  Arena::ISystem *m_p_system;
};

#endif // BUILD_CAMERAS_WRAPPER_H
