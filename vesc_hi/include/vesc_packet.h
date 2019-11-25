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
*********************************************************************/

/** NOTE *************************************************************
* This program had been developed by Michael T. Boulet at MIT under 
* the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank 
* Corp. takes over development as new packages.
*********************************************************************/

#ifndef VESC_DRIVER_VESC_PACKET_H_
#define VESC_DRIVER_VESC_PACKET_H_

#include <string>
#include <vector>
#include <utility>

#include <boost/crc.hpp>
#include <boost/shared_ptr.hpp>

#include "vesc_driver/v8stdint.h"

namespace vesc_driver
{

typedef std::vector<uint8_t> Buffer;
typedef std::pair<Buffer::iterator, Buffer::iterator> BufferRange;
typedef std::pair<Buffer::const_iterator, Buffer::const_iterator> BufferRangeConst;

/** The raw frame for communicating with the VESC */
class VescFrame
{
public:
  virtual ~VescFrame() {}

  // getters
  virtual const Buffer& frame() const {return *frame_;}

  // VESC packet properties
  static const int VESC_MAX_PAYLOAD_SIZE = 1024;          ///< Maximum VESC payload size, in bytes
  static const int VESC_MIN_FRAME_SIZE = 5;               ///< Smallest VESC frame size, in bytes
  static const int VESC_MAX_FRAME_SIZE = 6 + VESC_MAX_PAYLOAD_SIZE; ///< Largest VESC frame size, in bytes
  static const unsigned int VESC_SOF_VAL_SMALL_FRAME = 2; ///< VESC start of "small" frame value
  static const unsigned int VESC_SOF_VAL_LARGE_FRAME = 3; ///< VESC start of "large" frame value
  static const unsigned int VESC_EOF_VAL = 3;             ///< VESC end-of-frame value

  /** CRC parameters for the VESC */
  typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> CRC;

protected:
  /** Construct frame with specified payload size. */
  VescFrame(int payload_size);

  boost::shared_ptr<Buffer> frame_; ///< Stores frame data, shared_ptr for shallow copy
  BufferRange payload_;             ///< View into frame's payload section

private:
  /** Construct from buffer. Used by VescPacketFactory factory. */
  VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload);

  /** Give VescPacketFactory access to private constructor. */
  friend class VescPacketFactory;
};

/*------------------------------------------------------------------------------------------------*/

/** A VescPacket is a VescFrame with a non-zero length payload */
class VescPacket : public VescFrame
{
public:
  virtual ~VescPacket() {}

  virtual const std::string& name() const {return name_;}

protected:
  VescPacket(const std::string& name, int payload_size, int payload_id);
  VescPacket(const std::string& name, boost::shared_ptr<VescFrame> raw);

private:
  std::string name_;
};

typedef boost::shared_ptr<VescPacket> VescPacketPtr;
typedef boost::shared_ptr<VescPacket const> VescPacketConstPtr;

/*------------------------------------------------------------------------------------------------*/

class VescPacketFWVersion : public VescPacket
{
public:
  VescPacketFWVersion(boost::shared_ptr<VescFrame> raw);

  int fwMajor() const;
  int fwMinor() const;

};

class VescPacketRequestFWVersion : public VescPacket
{
public:
  VescPacketRequestFWVersion();

};

/*------------------------------------------------------------------------------------------------*/

class VescPacketValues : public VescPacket
{
public:
  VescPacketValues(boost::shared_ptr<VescFrame> raw);

  double v_in() const;
  double temp_mos1() const;
  double temp_mos2() const;
  double temp_mos3() const;
  double temp_mos4() const;
  double temp_mos5() const;
  double temp_mos6() const;
  double temp_pcb() const;
  double current_motor() const;
  double current_in() const;
  double rpm() const;
  double duty_now() const;
  double amp_hours() const;
  double amp_hours_charged() const;
  double watt_hours() const;
  double watt_hours_charged() const;
  double tachometer() const;
  double tachometer_abs() const;
  int fault_code() const;

};

class VescPacketRequestValues : public VescPacket
{
public:
  VescPacketRequestValues();
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetDuty : public VescPacket
{
public:
  VescPacketSetDuty(double duty);

  //  double duty() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetCurrent : public VescPacket
{
public:
  VescPacketSetCurrent(double current);

  //  double current() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetCurrentBrake : public VescPacket
{
public:
  VescPacketSetCurrentBrake(double current_brake);

  //  double current_brake() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetRPM : public VescPacket
{
public:
  VescPacketSetRPM(double rpm);

  //  double rpm() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetPos : public VescPacket
{
public:
  VescPacketSetPos(double pos);

  //  double pos() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetServoPos : public VescPacket
{
public:
  VescPacketSetServoPos(double servo_pos);

  //  double servo_pos() const;
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_PACKET_H_
