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



#include "arena_camera/arena_cameras_handler.h"
#include <algorithm>

ArenaCamerasHandler::ArenaCamerasHandler() {
  m_p_system = Arena::OpenSystem();
  m_p_system->UpdateDevices(100);
}

void ArenaCamerasHandler::create_cameras_from_settings(
    std::vector<CameraSetting> &camera_settings) {
  std::vector<Arena::DeviceInfo> devicesInfos = m_p_system->GetDevices();

  if (devicesInfos.size() == 0) {
    throw std::runtime_error("arena_camera: There is no connected devices.");
  }

  if (devicesInfos.size() < camera_settings.size()) {
    throw std::runtime_error("arena camera: Size of connected devices and "
                             "cameras in params mismatch.");
  }

  m_cameras.reserve(camera_settings.size());

  uint32_t camera_idx = 0;
  for (std::size_t i = 0; i < camera_settings.size(); ++i) {

    auto it = std::find_if(
        devicesInfos.begin(), devicesInfos.end(),
        [&](Arena::DeviceInfo &d_info) {
          return std::to_string(camera_settings.at(i).get_serial_no()) ==
                 d_info.SerialNumber().c_str();
        });

    if (it != devicesInfos.end()) {
      Arena::IDevice *device = m_p_system->CreateDevice(*it);
      m_cameras.emplace_back(device, camera_settings.at(i), camera_idx);
      camera_idx++;
    } else {
      throw std::runtime_error(
          "arena_camera: Wrong device serial no in parameters file.");
    }
  }
}

void ArenaCamerasHandler::set_image_callback(
    ArenaCamera::ImageCallbackFunction callback) {
  for (auto &camera : m_cameras) {
    camera.set_on_image_callback(callback);
  }
}

void ArenaCamerasHandler::start_stream() {

  std::vector<std::thread> threads;
  for (auto &camera : m_cameras) {
    threads.emplace_back(camera.start_stream());
  }
  for (auto &th : threads) {
    th.join();
  }
}

void ArenaCamerasHandler::stop_stream() {
  for (auto &camera : m_cameras) {
    camera.stop_stream();
  }
}

ArenaCamerasHandler::~ArenaCamerasHandler() {
  std::cout << " ~ArenaCamerasHandler()" << std::endl;
  for (auto &camera : m_cameras) {
    camera.destroy_device(m_p_system);
  }
  CloseSystem(m_p_system);
  m_cameras.clear();
}
