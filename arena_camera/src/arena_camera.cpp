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





#include "arena_camera/arena_camera.h"

ArenaCamera::ArenaCamera(Arena::IDevice *device, CameraSetting &camera_setting,
                         uint32_t camera_idx)
    : m_device(device), m_camera_name(camera_setting.get_camera_name()),
      m_frame_id(camera_setting.get_frame_id()),
      m_pixel_format(camera_setting.get_pixel_format()),
      m_serial_no(camera_setting.get_serial_no()),
      m_fps(camera_setting.get_fps()), m_cam_idx(camera_idx),
      m_width(camera_setting.get_width()),
      m_height(camera_setting.get_height()),
      m_flip_enable(camera_setting.get_flip_enable()),
      m_continue_acquiring(true) {
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

ArenaCamera::ArenaCamera(Arena::IDevice *device, std::string &camera_name,
                         std::string &frame_id, std::string &pixel_format,
                         uint32_t serial_no, uint32_t fps, uint32_t camera_idx,
                         uint32_t width, uint32_t height, bool flip_enable)
    : m_device(device), m_camera_name(camera_name), m_frame_id(frame_id),
      m_pixel_format(pixel_format), m_serial_no(serial_no), m_fps(fps),
      m_cam_idx(camera_idx), m_width(width), m_height(height),
      m_flip_enable(flip_enable) {
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

std::thread ArenaCamera::start_stream() {

  return std::thread([=] { this->acquisition(); });
}

void ArenaCamera::acquisition() {

  auto node_map = m_device->GetNodeMap();
  std::cout << "Camera idx:" << m_cam_idx << " acquisition thread."
            << std::endl;
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
                                         "AcquisitionMode", "Continuous");


  // enable stream auto negotiate packet size
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(),
                            "StreamAutoNegotiatePacketSize", true);

  //     enable stream packet resend
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(),
                            "StreamPacketResendEnable", true);

  auto max_fps =
      GenApi::CFloatPtr(node_map->GetNode("AcquisitionFrameRate"))->GetMax();
  if (m_fps > max_fps) {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate", max_fps);
  } else {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate",
                                static_cast<double>(m_fps));
  }

  m_device->StartStream();

  while (m_continue_acquiring) {
    try {
      pImage = m_device->GetImage(5000);

      if (pImage->IsIncomplete()) {
        m_device->RequeueBuffer(pImage);
        continue;
      }

      m_signal_publish_image(m_cam_idx, convert_to_image(pImage, m_frame_id));

      m_device->RequeueBuffer(pImage);
    } catch (GenICam::TimeoutException &ge) {
      std::cout << "GenICam exception thrown: " << ge.what() << std::endl;
    }
  }
}

void ArenaCamera::stop_stream() { m_device->StopStream(); }

void ArenaCamera::destroy_device(Arena::ISystem *system) {
  if (m_device != nullptr) {
    system->DestroyDevice(m_device);
  }
}

void ArenaCamera::set_on_image_callback(ImageCallbackFunction callback) {
  m_signal_publish_image = std::move(callback);
}

cv::Mat ArenaCamera::convert_to_image(Arena::IImage *pImage,
                                      const std::string &frame_id) {

  cv::Mat image_cv = cv::Mat(pImage->GetHeight(), pImage->GetWidth(), CV_8UC1,
                             (uint8_t *)pImage->GetData());

  cv::Mat image_bgr(image_cv.rows, image_cv.cols, CV_8UC3);
  cvtColor(image_cv, image_bgr, cv::COLOR_BayerBG2BGR);

  cv::resize(image_bgr, image_bgr, cv::Size(this->m_width, this->m_height));
  if (m_flip_enable) {
    cv::flip(image_bgr, image_bgr, -1);
  }

  return image_bgr;
}

ArenaCamera::~ArenaCamera() {
  std::cout << "Camera:" << m_cam_idx << " ~ArenaCamera()" << std::endl;
  m_continue_acquiring = false;
  stop_stream();
}
