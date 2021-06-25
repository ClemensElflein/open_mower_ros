/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#ifndef VESC_DRIVER_VESC_PACKET_FACTORY_H_
#define VESC_DRIVER_VESC_PACKET_FACTORY_H_

#include <cassert>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "vesc_driver/data_map.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{
/**
 * @brief Creates VESC packets from raw data.
 **/
class VescPacketFactory : private boost::noncopyable
{
public:
  static VescPacketPtr createPacket(const Buffer::const_iterator&, const Buffer::const_iterator&, int*, std::string*);

  typedef std::function<VescPacketPtr(std::shared_ptr<VescFrame>)> CreateFn;

  /** Register a packet type with the factory. */
  static void registerPacketType(int, CreateFn);

private:
  typedef std::map<int, CreateFn> FactoryMap;
  static FactoryMap* getMap();
};

/**
 * @def REGISTER_PACKET_TYPE
 * @brief Registers a type of the packet
 **/
#define REGISTER_PACKET_TYPE(id, klass)                                                                                \
  class klass##Factory                                                                                                 \
  {                                                                                                                    \
  public:                                                                                                              \
    klass##Factory()                                                                                                   \
    {                                                                                                                  \
      VescPacketFactory::registerPacketType((id), &klass##Factory::create);                                            \
    }                                                                                                                  \
    static VescPacketPtr create(std::shared_ptr<VescFrame> frame)                                                      \
    {                                                                                                                  \
      return VescPacketPtr(new klass(frame));                                                                          \
    }                                                                                                                  \
  };                                                                                                                   \
  static klass##Factory global_##klass##Factory;

}  // namespace vesc_driver

#endif  // VESC_DRIVER_VESC_PACKET_FACTORY_H_
