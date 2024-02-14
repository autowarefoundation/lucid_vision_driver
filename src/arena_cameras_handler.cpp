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
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

ArenaCamerasHandler::ArenaCamerasHandler()
{
  m_p_system = Arena::OpenSystem();
  m_p_system->UpdateDevices(100);
}

void ArenaCamerasHandler::create_camera_from_settings(CameraSetting & camera_settings)
{
  std::vector<Arena::DeviceInfo> devicesInfos = m_p_system->GetDevices();

  if (devicesInfos.size() == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "There is no connected devices. Please connect a device and try again.");
    throw std::runtime_error("arena_camera: There is no connected devices.");
  }

  auto it = std::find_if(devicesInfos.begin(), devicesInfos.end(), [&](Arena::DeviceInfo & d_info) {
    return std::to_string(camera_settings.get_serial_no()) == d_info.SerialNumber().c_str();
  });

  if (it != devicesInfos.end()) {
    m_device = m_p_system->CreateDevice(*it);
    m_enable_rectifying = camera_settings.get_enable_rectifying();
    m_enable_compressing = camera_settings.get_enable_compressing();
    m_use_default_device_settings = camera_settings.get_use_default_device_settings();
    // Prepare camera settings
    this->set_fps(camera_settings.get_fps());

    if (!m_use_default_device_settings) {
      this->set_auto_exposure(camera_settings.get_enable_exposure_auto());
      this->set_exposure_value(camera_settings.get_auto_exposure_value());
      this->set_auto_gain(camera_settings.get_enable_exposure_auto());
      this->set_gain_value(camera_settings.get_auto_gain_value());
      this->set_gamma_value(camera_settings.get_gamma_value());
      this->set_reverse_image_y(camera_settings.get_image_horizontal_flip());
      this->set_reverse_image_x(camera_settings.get_image_vertical_flip());
    }

    m_cameras = new ArenaCamera(m_device, camera_settings);
    m_device->RegisterImageCallback(m_cameras);

  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Wrong device serial no in parameters file. Please check the serial no and try again.");
    throw std::runtime_error("arena_camera: Wrong device serial no in parameters file.");
  }
}

void ArenaCamerasHandler::set_image_callback(ArenaCamera::ImageCallbackFunction callback)
{
  this->m_cameras->set_on_image_callback(callback);
}

void ArenaCamerasHandler::start_stream() { this->m_cameras->acquisition(); }

void ArenaCamerasHandler::stop_stream() { this->m_cameras->stop_stream(); }

void ArenaCamerasHandler::set_fps(uint32_t fps)
{
  auto node_map = m_device->GetNodeMap();
  auto max_fps = GenApi::CFloatPtr(node_map->GetNode("AcquisitionFrameRate"))->GetMax();
  if (fps > max_fps || fps < 0) {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate", max_fps);
  } else {
    Arena::SetNodeValue<bool>(node_map, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(node_map, "AcquisitionFrameRate", static_cast<double>(fps));
  }
}

ArenaCamerasHandler::~ArenaCamerasHandler()
{
  std::cout << " ~ArenaCamerasHandler()" << std::endl;

  this->stop_stream();
  m_device->DeregisterImageCallback(m_cameras);
  this->m_cameras->destroy_device(m_p_system);
  CloseSystem(m_p_system);

  delete m_cameras;
  delete m_p_system;
  delete m_device;
}

GenICam_3_3_LUCID::gcstring ArenaCamerasHandler::get_auto_exposure()
{
  return Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto");
}

void ArenaCamerasHandler::set_auto_exposure(bool auto_exposure)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto exposure. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring exposure_auto = auto_exposure ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto", exposure_auto);
}

void ArenaCamerasHandler::set_exposure_value(float exposure_value)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure value. Using default device settings.");
    return;
  }

  const auto auto_exposure = this->get_auto_exposure();
  if (auto_exposure == "Off") {
    GenApi::CFloatPtr pExposureTime = m_device->GetNodeMap()->GetNode("ExposureTime");
    try {
      if (exposure_value < pExposureTime->GetMin()) {
        exposure_value = pExposureTime->GetMin();

      } else if (exposure_value > pExposureTime->GetMax()) {
        exposure_value = pExposureTime->GetMax();
      }
      pExposureTime->SetValue(exposure_value);
    } catch (const GenICam::GenericException & e) {
      std::cerr << "Exception occurred during exposure value handling: " << e.GetDescription()
                << std::endl;
    }
  }else{
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure value when auto exposure is enabled.");
  }
}
GenICam_3_3_LUCID::gcstring ArenaCamerasHandler::get_auto_gain()
{
  return Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "GainAuto");
}

void ArenaCamerasHandler::set_auto_gain(bool auto_gain)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto gain. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring gain_auto = auto_gain ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "GainAuto", gain_auto);
}

void ArenaCamerasHandler::set_gain_value(float gain_value)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }
  const auto auto_gain = this->get_auto_gain();
  if (auto_gain == "Off") {
    GenApi::CFloatPtr pGain = m_device->GetNodeMap()->GetNode("Gain");
    try {
      if (gain_value < pGain->GetMin()) {
        gain_value = pGain->GetMin();
      } else if (gain_value > pGain->GetMax()) {
        gain_value = pGain->GetMax();
      }
      pGain->SetValue(gain_value);
    } catch (const GenICam::GenericException & e) {
      std::cerr << "Exception occurred during gain value handling: " << e.GetDescription()
                << std::endl;
    }
  }else{
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value when auto gain is enabled.");
  }
}
void ArenaCamerasHandler::set_gamma_value(float gamma_value)
{

  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }

  try {
    GenApi::CBooleanPtr pGammaEnable = m_device->GetNodeMap()->GetNode("GammaEnable");
    if (GenApi::IsWritable(pGammaEnable)) {
      pGammaEnable->SetValue(true);
    }
    GenApi::CFloatPtr pGamma = m_device->GetNodeMap()->GetNode("Gamma");
    if (pGamma && GenApi::IsWritable(pGamma)) {
      if (pGamma->GetMin() > gamma_value) {
        gamma_value = pGamma->GetMin();
      } else if (pGamma->GetMax() < gamma_value) {
        gamma_value = pGamma->GetMax();
      }
      pGamma->SetValue(gamma_value);
    }
  } catch (const GenICam::GenericException & e) {
    std::cerr << "Exception occurred during gamma value handling: " << e.GetDescription()
              << std::endl;
  }
}
void ArenaCamerasHandler::set_enable_rectifying(bool enable_rectifying)
{
  this->m_enable_rectifying = enable_rectifying;
}
bool ArenaCamerasHandler::get_enable_rectifying()
{
  return m_enable_rectifying;
}
void ArenaCamerasHandler::set_enable_compressing(bool enable_compressing)
{
  this->m_enable_compressing = enable_compressing;
}
bool ArenaCamerasHandler::get_enable_compressing()
{
  return m_enable_compressing;
}
void ArenaCamerasHandler::set_use_default_device_settings(bool use_default_device_settings)
{
  this->m_use_default_device_settings = use_default_device_settings;
}
bool ArenaCamerasHandler::get_use_default_device_settings()
{
  return m_use_default_device_settings;
}
void ArenaCamerasHandler::set_reverse_image_y(bool image_horizontal_flip)
{

  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set horizontal flip. Using default device settings.");
    return;
  }

  try {
    GenApi::CBooleanPtr pHorizontalFlip = m_device->GetNodeMap()->GetNode("ReverseY");
    if (GenApi::IsWritable(pHorizontalFlip)) {
      pHorizontalFlip->SetValue(image_horizontal_flip);
    }
  } catch (const GenICam::GenericException & e) {
    std::cerr << "Exception occurred during ReverseY value handling: " << e.GetDescription()
              << std::endl;
  }
}

void ArenaCamerasHandler::set_reverse_image_x(bool image_vertical_flip)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set vertical flip. Using default device settings.");
    return;
  }
  try {
    GenApi::CBooleanPtr pVerticalFlip = m_device->GetNodeMap()->GetNode("ReverseX");
    if (GenApi::IsWritable(pVerticalFlip)) {
      pVerticalFlip->SetValue(image_vertical_flip);
    }
  } catch (const GenICam::GenericException & e) {
    std::cerr << "Exception occurred during ReverseX value handling: " << e.GetDescription()
              << std::endl;
  }
}