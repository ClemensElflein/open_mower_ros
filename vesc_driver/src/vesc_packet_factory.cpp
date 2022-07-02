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

#include "vesc_driver/vesc_packet_factory.h"

namespace vesc_driver
{
/**
 * @brief Creates failure message when createPacket can not create a packet
 **/
VescPacketPtr createFailed(int* p_num_bytes_needed, std::string* p_what, const std::string& what,
                           const int num_bytes_needed = 0)
{
  if (p_num_bytes_needed != NULL)
    *p_num_bytes_needed = num_bytes_needed;
  if (p_what != NULL)
    *p_what = what;
  return VescPacketPtr();
}

/**
 * @brief Creates a VESC packet
 * @details Create a VescPacket from a buffer (factory function). Packet must
 * start @p begin and complete (end of frame character) before what @p begin
 * points to end. The buffer element @p end is not examined, i.e. it can be the
 * past-the-end element. This function only returns a packet if the packet is
 * valid, i.e. valid size, matching checksum, complete etc. An empty pointer
 * is returned if a packet cannot be found or if it is invalid. If a valid
 * packet is not found, an optional output parameter @p what is set to a string
 * providing a reason why a packet was not found. If a packet was not found
 * because additional bytes are needed on the buffer, optional output parameter
 * @p num_bytes_needed will contain the number of bytes needed to either
 * determine the size of the packet or complete the packet. Output parameters
 * @p num_bytes_needed and @p what will be set to 0 and empty if a valid packet
 * is found.
 *
 * @param begin[in] Iterator to a buffer at the start-of-frame character.
 * @param end[in] Iterator to the buffer past-the-end element.
 * @param num_bytes_needed[out] Number of bytes needed to determine the packet
 *  size or complete the frame.
 * @param what[out] Message string giving a reason why the packet was not found.
 * @return Pointer to a valid VescPacket if successful; otherwise, an empty
 * pointer.
 **/
VescPacketPtr VescPacketFactory::createPacket(const Buffer::const_iterator& begin, const Buffer::const_iterator& end,
                                              int* num_bytes_needed, std::string* what)
{
  // initializes output variables
  if (num_bytes_needed != NULL)
  {
    *num_bytes_needed = 0;
  }
  if (what != NULL)
  {
    what->clear();
  }

  // requires at least VESC_MIN_FRAME_SIZE bytes in buffer
  int buffer_size(std::distance(begin, end));
  if (buffer_size < VescFrame::VESC_MIN_FRAME_SIZE)
  {
    return createFailed(num_bytes_needed, what, "Buffer does not contain a complete frame",
                        VescFrame::VESC_MIN_FRAME_SIZE - buffer_size);
  }

  // checks whether buffer begins with a start-of-frame
  if (VescFrame::VESC_SOF_VAL_SMALL_FRAME != *begin && VescFrame::VESC_SOF_VAL_LARGE_FRAME != *begin)
  {
    return createFailed(num_bytes_needed, what, "Buffer must begin with start-of-frame character");
  }

  // gets a view of the payload
  BufferRangeConst view_payload;
  if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *begin)
  {
    // payload size field is one byte
    view_payload.first = begin + 2;
    view_payload.second = view_payload.first + *(begin + 1);
  }
  else
  {
    assert(VescFrame::VESC_SOF_VAL_LARGE_FRAME == *begin);
    // payload size field is two bytes
    view_payload.first = begin + 3;
    view_payload.second = view_payload.first + (*(begin + 1) << 8) + *(begin + 2);
  }

  // checks the length
  if (boost::distance(view_payload) > VescFrame::VESC_MAX_PAYLOAD_SIZE)
  {
    return createFailed(num_bytes_needed, what, "Invalid payload length");
  }

  // gets iterators to crc field, end-of-frame field, and a view of the whole frame
  Buffer::const_iterator iter_crc(view_payload.second);
  Buffer::const_iterator iter_eof(iter_crc + 2);
  BufferRangeConst view_frame(begin, iter_eof + 1);

  // chekcs whether enough data is loaded in the buffer to complete the frame
  int frame_size = boost::distance(view_frame);
  if (buffer_size < frame_size)
    return createFailed(num_bytes_needed, what, "Buffer does not contain a complete frame", frame_size - buffer_size);

  // checks whether the end-of-frame character is valid
  if (VescFrame::VESC_EOF_VAL != *iter_eof)
    return createFailed(num_bytes_needed, what, "Invalid end-of-frame character");

  // checks whether the crc is valid
  uint16_t crc = (static_cast<uint16_t>(*iter_crc) << 8) + *(iter_crc + 1);
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*view_payload.first), boost::distance(view_payload));
  if (crc != crc_calc.checksum())
    return createFailed(num_bytes_needed, what, "Invalid checksum");

  // constructs the raw frame
  std::shared_ptr<VescFrame> raw_frame(new VescFrame(view_frame, view_payload));

  // constructs the corresponding subclass if the packet has a payload
  if (boost::distance(view_payload) > 0)
  {
    // gets constructor function from payload id
    FactoryMap* p_map(getMap());
    FactoryMap::const_iterator search(p_map->find(*view_payload.first));

    if (search != p_map->end())
    {
      return search->second(raw_frame);
    }
    else
    {
      // no subclass constructor for this packet
      return createFailed(num_bytes_needed, what, "Unkown payload type.");
    }
  }
  else
  {
    // no payload
    return createFailed(num_bytes_needed, what, "Frame does not have a payload");
  }
}

/**
 * @brief Registers a type of the packet
 * @param payload_id Payload ID
 * @param fn Function pointer
 **/
void VescPacketFactory::registerPacketType(int payload_id, CreateFn fn)
{
  FactoryMap* p_map(getMap());
  assert(0 == p_map->count(payload_id));
  (*p_map)[payload_id] = fn;
}

/**
 * @brief Constructa a map on first use
 * @return Pointer to the constructed map
 **/
VescPacketFactory::FactoryMap* VescPacketFactory::getMap()
{
  static FactoryMap m;
  return &m;
}

REGISTER_PACKET_TYPE(COMM_FW_VERSION, VescPacketFWVersion)
REGISTER_PACKET_TYPE(COMM_GET_VALUES, VescPacketValues)

}  // namespace xesc_2040_driver
