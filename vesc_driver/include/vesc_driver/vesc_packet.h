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
#include <cstdint>
#include <cassert>
#include <iterator>

#include <boost/crc.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "vesc_driver/datatypes.h"

namespace vesc_driver {

typedef std::vector<uint8_t> Buffer;
typedef std::pair<Buffer::iterator, Buffer::iterator>             BufferRange;
typedef std::pair<Buffer::const_iterator, Buffer::const_iterator> BufferRangeConst;

/**
 * @brief The raw frame for communicating with the VESC
 **/
class VescFrame {
public:
    /**
     * @brief Destructor
     **/
    virtual ~VescFrame() {
    }

    /**
     * @brief Gets a reference of the frame
     * @return Reference of the frame
     **/
    virtual const Buffer& getFrame() const {
        return frame_;
    }

    /* packet properties */
    static const int16_t VESC_MAX_PAYLOAD_SIZE    = 1024;                       // Maximum payload size (bytes)
    static const int16_t VESC_MIN_FRAME_SIZE      = 5;                          // Smallest frame size (bytes)
    static const int16_t VESC_MAX_FRAME_SIZE      = 6 + VESC_MAX_PAYLOAD_SIZE;  // Largest frame size (bytes)
    static const int16_t VESC_SOF_VAL_SMALL_FRAME = 2;                          // Start of "small" frame value
    static const int16_t VESC_SOF_VAL_LARGE_FRAME = 3;                          // Start of "large" frame value
    static const int16_t VESC_EOF_VAL             = 3;                          // End-of-frame value

    /**
     * @brief CRC parameters for the VESC
     **/
    typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> CRC;

protected:
    explicit VescFrame(const int16_t payload_size);

    Buffer      frame_;    // Stores frame data, shared_ptr for shallow copy
    BufferRange payload_;  // View into frame's payload section

private:
    VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload);

    friend class VescPacketFactory;  // gives VescPacketFactory access to private constructor
};

/*------------------------------------------------------------------*/

/**
 * @brief VescFrame with a non-zero length payload
 **/
class VescPacket : public VescFrame {
public:
    /**
     * @brief Destructor
     **/
    virtual ~VescPacket() {
    }

    /**
     * @brief Gets the packet name
     * @return The packet name
     **/
    virtual const std::string& getName() const {
        return name_;
    }

protected:
    VescPacket(const std::string& name, const int16_t payload_size, const int16_t payload_id);
    VescPacket(const std::string& name, boost::shared_ptr<VescFrame> raw);

private:
    std::string name_;
};

typedef boost::shared_ptr<VescPacket>       VescPacketPtr;
typedef boost::shared_ptr<VescPacket const> VescPacketConstPtr;

/*------------------------------------------------------------------*/

/**
 * @brief Farmware version
 **/
class VescPacketFWVersion : public VescPacket {
public:
    explicit VescPacketFWVersion(boost::shared_ptr<VescFrame> raw);

    int16_t fwMajor() const;
    int16_t fwMinor() const;
};

/*------------------------------------------------------------------*/

/**
 * @brief Requests farmware version
 **/
class VescPacketRequestFWVersion : public VescPacket {
public:
    VescPacketRequestFWVersion();
};

/*------------------------------------------------------------------*/

/**
 * @brief Map of return packets
 **/
enum PACKET_MAP {
    TEMP_MOS           = 1,
    TEMP_MOTOR         = 3,
    CURRENT_MOTOR      = 5,
    CURRENT_IN         = 9,
    ID                 = 13,
    IQ                 = 17,
    DUTY_NOW           = 21,
    RPM                = 23,
    VOLTAGE_IN         = 27,
    AMP_HOURS          = 29,
    AMP_HOURS_CHARGED  = 33,
    WATT_HOURS         = 37,
    WATT_HOURS_CHARGED = 41,
    TACHOMETER         = 45,
    TACHOMETER_ABS     = 49,
    FAULT_CODE         = 53,
};

/**
 * @brief Gets values in return packets
 **/
class VescPacketValues : public VescPacket {
public:
    explicit VescPacketValues(boost::shared_ptr<VescFrame> raw);

    double temp_mos() const;
    double temp_motor() const;
    double current_motor() const;
    double current_in() const;
    double rpm() const;
    double v_in() const;
    double duty_now() const;
    double amp_hours() const;
    double amp_hours_charged() const;
    double watt_hours() const;
    double watt_hours_charged() const;
    double tachometer() const;
    double tachometer_abs() const;
    int    fault_code() const;

private:
    double readBuffer(const uint8_t, const uint8_t) const;
};

/*------------------------------------------------------------------*/

class VescPacketRequestValues : public VescPacket {
public:
    VescPacketRequestValues();
};

/*------------------------------------------------------------------*/

class VescPacketSetDuty : public VescPacket {
public:
    explicit VescPacketSetDuty(double duty);

    //  double duty() const;
};

/*------------------------------------------------------------------*/

class VescPacketSetCurrent : public VescPacket {
public:
    explicit VescPacketSetCurrent(double current);

    //  double current() const;
};

/*------------------------------------------------------------------*/

class VescPacketSetCurrentBrake : public VescPacket {
public:
    explicit VescPacketSetCurrentBrake(double current_brake);

    //  double current_brake() const;
};

/*------------------------------------------------------------------*/

class VescPacketSetRPM : public VescPacket {
public:
    explicit VescPacketSetRPM(double rpm);

    //  double rpm() const;
};

/*------------------------------------------------------------------*/

class VescPacketSetPos : public VescPacket {
public:
    explicit VescPacketSetPos(double pos);

    //  double pos() const;
};

/*------------------------------------------------------------------*/

class VescPacketSetServoPos : public VescPacket {
public:
    explicit VescPacketSetServoPos(double servo_pos);

    //  double servo_pos() const;
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER_VESC_PACKET_H_
