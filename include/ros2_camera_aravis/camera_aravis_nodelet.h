/****************************************************************************
 *
 * camera_aravis
 *
 * Copyright © 2019 Fraunhofer FKIE, Straw Lab, van Breugel Lab, and contributors
 * Authors: Dominik A. Klein,
 * 			Floris van Breugel,
 * 			Andrew Straw,
 * 			Steve Safarik
 *
 * Licensed under the LGPL, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.gnu.org/licenses/lgpl-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef INCLUDE_CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET_H_
#define INCLUDE_CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET_H_

extern "C" {
  #include <arv.h>
}

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>
#include <algorithm>
#include <functional>
#include <cctype>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <unordered_map>

#include <glib.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int64.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ros2_camera_aravis/srv/get_integer_feature.hpp>
#include <ros2_camera_aravis/srv/set_integer_value.hpp>
#include <ros2_camera_aravis/srv/get_float_feature.hpp>
#include <ros2_camera_aravis/srv/set_float_value.hpp>
#include <ros2_camera_aravis/srv/get_string_feature.hpp>
#include <ros2_camera_aravis/srv/set_string_feature.hpp>
#include <ros2_camera_aravis/srv/get_boolean_feature.hpp>
#include <ros2_camera_aravis/srv/set_boolean_value.hpp>

#include "camera_buffer_pool.h"

namespace camera_aravis
{

typedef CameraAravisConfig Config;

// Conversions from integers to Arv types.
const char *szBufferStatusFromInt[] = {"ARV_BUFFER_STATUS_SUCCESS", "ARV_BUFFER_STATUS_CLEARED",
                                       "ARV_BUFFER_STATUS_TIMEOUT", "ARV_BUFFER_STATUS_MISSING_PACKETS",
                                       "ARV_BUFFER_STATUS_WRONG_PACKET_ID", "ARV_BUFFER_STATUS_SIZE_MISMATCH",
                                       "ARV_BUFFER_STATUS_FILLING", "ARV_BUFFER_STATUS_ABORTED"};

// Conversion functions from Genicam to ROS formats
typedef std::function<void(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out)> ConversionFunction;
void renameImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);
void shiftImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const size_t n_digits, const std::string out_format);
void interleaveImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const size_t n_digits, const std::string out_format);
void unpack10p32Img(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);
void unpack10PackedImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);
void unpack10pMonoImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);
void unpack10PackedMonoImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);
void unpack12pImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);
void unpack12PackedImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);
void unpack565pImg(std::shared_ptr<sensor_msgs::msg::Image>& in, std::shared_ptr<sensor_msgs::msg::Image>& out, const std::string out_format);

const std::map<std::string, ConversionFunction> CONVERSIONS_DICTIONARY =
{
 // equivalent to official ROS color encodings
 { "RGB8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB8) },
 { "RGBa8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGBA8) },
 { "RGB16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGBa16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGBA16) },
 { "BGR8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGR8) },
 { "BGRa8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGRA8) },
 { "BGR16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGR16) },
 { "BGRa16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGRA16) },
 { "Mono8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "Raw8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "R8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "G8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "B8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO8) },
 { "Mono16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "Raw16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "R16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "G16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "B16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "BayerRG8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB8) },
 { "BayerBG8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR8) },
 { "BayerGB8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG8) },
 { "BayerGR8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG8) },
 { "BayerRG16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "YUV422_8_UYVY", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::YUV422) },
 { "YUV422_8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::YUV422) },
 // non-color contents
 { "Data8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_8UC1) },
 { "Confidence8", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_8UC1) },
 { "Data8s", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_8SC1) },
 { "Data16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_16UC1) },
 { "Confidence16", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_16UC1) },
 { "Data16s", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_16SC1) },
 { "Data32s", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_32SC1) },
 { "Data32f", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_32FC1) },
 { "Confidence32f", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_32FC1) },
 { "Data64f", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::TYPE_64FC1) },
 // unthrifty formats. Shift away padding Bits for use with ROS.
 { "Mono10", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::MONO16) },
 { "Mono12", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::MONO16) },
 { "Mono14", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 2, sensor_msgs::image_encodings::MONO16) },
 { "RGB10", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::RGB16) },
 { "RGB12", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::RGB16) },
 { "BGR10", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::BGR16) },
 { "BGR12", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::BGR16) },
 { "BayerRG10", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG10", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB10", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR10", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "BayerRG12", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG12", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB12", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR12", std::bind(&shiftImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::BAYER_GRBG16) },
 // planar instead pixel-by-pixel encodings
 { "RGB8_Planar", std::bind(&interleaveImg, std::placeholders::_1, std::placeholders::_2, 0, sensor_msgs::image_encodings::RGB8) },
 { "RGB10_Planar", std::bind(&interleaveImg, std::placeholders::_1, std::placeholders::_2, 6, sensor_msgs::image_encodings::RGB16) },
 { "RGB12_Planar", std::bind(&interleaveImg, std::placeholders::_1, std::placeholders::_2, 4, sensor_msgs::image_encodings::RGB16) },
 { "RGB16_Planar", std::bind(&interleaveImg, std::placeholders::_1, std::placeholders::_2, 0, sensor_msgs::image_encodings::RGB16) },
 // packed, non-Byte aligned formats
 { "Mono10p", std::bind(&unpack10pMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "RGB10p", std::bind(&unpack10p32Img, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGB10p32", std::bind(&unpack10p32Img, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGBa10p", std::bind(&unpack10p32Img, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGBA16) },
 { "BGR10p", std::bind(&unpack10p32Img, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGR16) },
 { "BGRa10p", std::bind(&unpack10p32Img, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGRA16) },
 { "BayerRG10p", std::bind(&unpack10pMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG10p", std::bind(&unpack10pMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB10p", std::bind(&unpack10pMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR10p", std::bind(&unpack10pMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "Mono12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "RGB12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGBa12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGBA16) },
 { "BGR12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGR16) },
 { "BGRa12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGRA16) },
 { "BayerRG12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR12p", std::bind(&unpack12pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "RGB565p", std::bind(&unpack565pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB8) },
 { "BGR565p", std::bind(&unpack565pImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BGR8) },
 // GigE-Vision specific format naming
 { "RGB10V1Packed", std::bind(&unpack10PackedImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGB10V2Packed", std::bind(&unpack10p32Img, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "RGB12V1Packed", std::bind(&unpack12PackedImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::RGB16) },
 { "Mono10Packed", std::bind(&unpack10PackedMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "Mono12Packed", std::bind(&unpack12PackedImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::MONO16) },
 { "BayerRG10Packed", std::bind(&unpack10PackedMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG10Packed", std::bind(&unpack10PackedMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB10Packed", std::bind(&unpack10PackedMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR10Packed", std::bind(&unpack10PackedMonoImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "BayerRG12Packed", std::bind(&unpack12PackedImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_RGGB16) },
 { "BayerBG12Packed", std::bind(&unpack12PackedImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_BGGR16) },
 { "BayerGB12Packed", std::bind(&unpack12PackedImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GBRG16) },
 { "BayerGR12Packed", std::bind(&unpack12PackedImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::BAYER_GRBG16) },
 { "YUV422Packed", std::bind(&renameImg, std::placeholders::_1, std::placeholders::_2, sensor_msgs::image_encodings::YUV422) }
};

class CameraAravisNodelet : public nodelet::Nodelet
{
public:
  CameraAravisNodelet();
  virtual ~CameraAravisNodelet();

private:
  virtual void onInit() override;
  void spawnStream();

protected:
  // reset PTP clock
  void resetPtpClock();

  // apply auto functions from a ros message
  void cameraAutoInfoCallback(const CameraAutoInfoConstPtr &msg_ptr);

  void syncAutoParameters();
  void setAutoMaster(bool value);
  void setAutoSlave(bool value);

  void setExtendedCameraInfo(std::string channel_name, size_t stream_id);
  void fillExtendedCameraInfoMessage(ExtendedCameraInfo &msg);

  // Extra stream options for GigEVision streams.
  void tuneGvStream(ArvGvStream *p_stream);

  void rosReconfigureCallback(Config &config, uint32_t level);

  // Start and stop camera on demand
  void rosConnectCallback();

  // Callback to wrap and send recorded image as ROS message
  static void newBufferReadyCallback(ArvStream *p_stream, gpointer can_instance);

  // Buffer Callback Helper
  static void newBufferReady(ArvStream *p_stream, CameraAravisNodelet *p_can, std::string frame_id, size_t stream_id);

  // Clean-up if aravis device is lost
  static void controlLostCallback(ArvDevice *p_gv_device, gpointer can_instance);

  // Services
  ros::ServiceServer get_integer_service_;
  bool getIntegerFeatureCallback(camera_aravis::get_integer_feature_value::Request& request, camera_aravis::get_integer_feature_value::Response& response);

  ros::ServiceServer get_float_service_;
  bool getFloatFeatureCallback(camera_aravis::get_float_feature_value::Request& request, camera_aravis::get_float_feature_value::Response& response);

  ros::ServiceServer get_string_service_;
  bool getStringFeatureCallback(camera_aravis::get_string_feature_value::Request& request, camera_aravis::get_string_feature_value::Response& response);

  ros::ServiceServer get_boolean_service_;
  bool getBooleanFeatureCallback(camera_aravis::get_boolean_feature_value::Request& request, camera_aravis::get_boolean_feature_value::Response& response);

  ros::ServiceServer set_integer_service_;
  bool setIntegerFeatureCallback(camera_aravis::set_integer_feature_value::Request& request, camera_aravis::set_integer_feature_value::Response& response);

  ros::ServiceServer set_float_service_;
  bool setFloatFeatureCallback(camera_aravis::set_float_feature_value::Request& request, camera_aravis::set_float_feature_value::Response& response);

  ros::ServiceServer set_string_service_;
  bool setStringFeatureCallback(camera_aravis::set_string_feature_value::Request& request, camera_aravis::set_string_feature_value::Response& response);

  ros::ServiceServer set_boolean_service_;
  bool setBooleanFeatureCallback(camera_aravis::set_boolean_feature_value::Request& request, camera_aravis::set_boolean_feature_value::Response& response);

  // triggers a shot at regular intervals, sleeps in between
  void softwareTriggerLoop();

  void publishTfLoop(double rate);

  void discoverFeatures();

  static void parseStringArgs(std::string in_arg_string, std::vector<std::string> &out_args);

  // WriteCameraFeaturesFromRosparam()
  // Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
  // camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
  // in the camera.
  //
  // Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
  // looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
  // integers are integers, doubles are doubles, etc.
  void writeCameraFeaturesFromRosparam();

  // std::unique_ptr<dynamic_reconfigure::Server<Config> > reconfigure_server_;
  std::recursive_mutex reconfigure_mutex_;

  std::vector<image_transport::CameraPublisher> cam_pubs_;
  std::vector<std::unique_ptr<camera_info_manager::CameraInfoManager>> p_camera_info_managers_;
  std::vector<std::unique_ptr<ros::NodeHandle>> p_camera_info_node_handles_;
  std::vector<sensor_msgs::CameraInfoPtr> camera_infos_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> p_stb_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> p_tb_;
  geometry_msgs::TransformStamped tf_optical_;
  std::thread tf_dyn_thread_;
  std::atomic_bool tf_thread_active_;

  CameraAutoInfo auto_params_;
  ros::Publisher auto_pub_;
  ros::Subscriber auto_sub_;

  std::recursive_mutex extended_camera_info_mutex_;
  std::vector<ros::Publisher> extended_camera_info_pubs_;

  Config config_;
  Config config_min_;
  Config config_max_;  
  bool   use_ptp_stamp_;

  std::atomic<bool> spawning_;
  std::thread       spawn_stream_thread_;

  std::thread software_trigger_thread_;
  std::atomic_bool software_trigger_active_;
  size_t n_buffers_ = 0;

  std::unordered_map<std::string, const bool> implemented_features_;

  struct
  {
    int32_t x = 0;
    int32_t y = 0;
    int32_t width = 0;
    int32_t width_min = 0;
    int32_t width_max = 0;
    int32_t height = 0;
    int32_t height_min = 0;
    int32_t height_max = 0;
  } roi_;

  struct Sensor
  {
    int32_t width = 0;
    int32_t height = 0;
    std::string pixel_format;
    size_t n_bits_pixel = 0;
  };

  std::vector<Sensor *> sensors_;

  struct StreamIdData
  {
    CameraAravisNodelet* can;
    size_t stream_id;
  };

  ArvCamera *p_camera_ = NULL;
  ArvDevice *p_device_ = NULL;
  gint num_streams_;
  std::vector<ArvStream *> p_streams_;
  std::vector<std::string> stream_names_;
  bool extended_camera_info_;
  std::vector<CameraBufferPool::Ptr> p_buffer_pools_;
  int32_t acquire_ = 0;
  std::vector<ConversionFunction> convert_formats;
};

} // end namespace camera_aravis

#endif /* INCLUDE_CAMERA_ARAVIS_CAMERA_ARAVIS_NODELET_H_ */
