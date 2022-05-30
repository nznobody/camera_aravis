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

#ifndef INCLUDE_CAMERA_ARAVIS_CAMERA_BUFFER_POOL_H_
#define INCLUDE_CAMERA_ARAVIS_CAMERA_BUFFER_POOL_H_

#include <arv.h>

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>
#include <type_traits>
#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"

#include <mutex>
#include <map>
#include <stack>

namespace camera_aravis
{

class CameraBufferPool : public std::enable_shared_from_this<CameraBufferPool>
{
public:
  typedef std::shared_ptr<CameraBufferPool> Ptr;
  typedef std::weak_ptr<CameraBufferPool> WPtr;

  // Note: If the CameraBufferPool is destroyed, buffers will be deallocated. Therefor, make sure
  // that the CameraBufferPool stays alive longer than the given stream object.
  //
  // stream: 			weakly managed pointer to the stream. Used to register all allocated buffers
  // payload_size_bytes:	size of a single buffer
  // n_preallocated_buffers:	number of initially allocated and registered buffers
  CameraBufferPool(ArvStream *stream, size_t payload_size_bytes, size_t n_preallocated_buffers = 2);
  virtual ~CameraBufferPool();

  // Get an image whose lifespan is administrated by this pool (but not registered to the camera).
  std::shared_ptr<sensor_msgs::msg::Image> getRecyclableImg();

  // Get the image message which wraps around the given ArvBuffer.
  //
  // If this buffer is not administrated by this CameraBufferPool,
  // a new image message is allocated and the contents of the buffer
  // are copied to it.
  std::shared_ptr<sensor_msgs::msg::Image> operator[](ArvBuffer *buffer);

  inline size_t getAllocatedSize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return n_buffers_;
  }

  inline size_t getUsedSize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return used_buffers_.size();
  }

  inline size_t getPayloadSize() const
  {
    return payload_size_bytes_;
  }

  // Allocate new buffers which are wrapped by an image message and
  // push them to the internal aravis stream.
  void allocateBuffers(size_t n = 1);

protected:
  // Custom deleter for aravis buffer wrapping image messages, which
  // either pushes the buffer back to the aravis stream cleans it up
  // when the CameraBufferPool is gone.
  static void reclaim(const WPtr &self, std::shared_ptr<sensor_msgs::msg::Image> p_img);

  // Push the buffer inside the given image back to the aravis stream,
  // remember the corresponding image message.
  void push(std::shared_ptr<sensor_msgs::msg::Image> p_img);

  ArvStream *stream_ = NULL;
  size_t payload_size_bytes_ = 0;
  size_t n_buffers_ = 0;

  std::map<const uint8_t*, std::shared_ptr<sensor_msgs::msg::Image>> available_img_buffers_;
  std::map<std::shared_ptr<sensor_msgs::msg::Image>, ArvBuffer*> used_buffers_;
  std::stack<std::shared_ptr<sensor_msgs::msg::Image>> dangling_imgs_;
  mutable std::mutex mutex_;
  Ptr self_;
};

} /* namespace camera_aravis */

#endif /* INCLUDE_CAMERA_ARAVIS_CAMERA_BUFFER_POOL_H_ */
