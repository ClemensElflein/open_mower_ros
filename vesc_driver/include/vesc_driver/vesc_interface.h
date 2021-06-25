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

#ifndef VESC_DRIVER_VESC_INTERFACE_H_
#define VESC_DRIVER_VESC_INTERFACE_H_

#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <pthread.h>
#include <serial/serial.h>
#include <boost/crc.hpp>
#include <boost/noncopyable.hpp>

#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_packet_factory.h"

namespace vesc_driver
{
/**
 * Class providing an interface to the Vedder VESC motor controller via a serial port interface.
 */
class VescInterface : private boost::noncopyable
{
public:
  typedef std::function<void(const VescPacketConstPtr&)> PacketHandlerFunction;
  typedef std::function<void(const std::string&)> ErrorHandlerFunction;

  /**
   * Creates a VescInterface object. Opens the serial port interface to the VESC if @p port is not
   * empty, otherwise the serial port remains closed until connect() is called.
   *
   * @param port Address of the serial port, e.g. '/dev/ttyUSB0'.
   * @param packet_handler Function this class calls when a VESC packet is received.
   * @param error_handler Function this class calls when an error is detected, such as a bad
   *                      checksum.
   *
   * @throw SerialException
   */
  VescInterface(const std::string& port = std::string(),
                const PacketHandlerFunction& packet_handler = PacketHandlerFunction(),
                const ErrorHandlerFunction& error_handler = ErrorHandlerFunction());

  /**
   * VescInterface destructor.
   */
  ~VescInterface();

  /**
   * Sets / updates the function that this class calls when a VESC packet is received.
   */
  void setPacketHandler(const PacketHandlerFunction& handler);

  /**
   * Sets / updates the function that this class calls when an error is detected, such as a bad
   * checksum.
   */
  void setErrorHandler(const ErrorHandlerFunction& handler);

  /**
   * Opens the serial port interface to the VESC.
   *
   * @throw SerialException
   */
  void connect(const std::string& port);

  /**
   * Closes the serial port interface to the VESC.
   */
  void disconnect();

  /**
   * Gets the status of the serial interface to the VESC.
   *
   * @return Returns true if the serial port is open, false otherwise.
   */
  bool isConnected() const;

  /**
   * Send a VESC packet.
   */
  void send(const VescPacket& packet);

  void requestFWVersion();
  void requestState();
  void setDutyCycle(double duty_cycle);
  void setCurrent(double current);
  void setBrake(double brake);
  void setSpeed(double speed);
  void setPosition(double position);
  void setServo(double servo);

private:
  // Pimpl - hide serial port members from class users
  class Impl;
  std::unique_ptr<Impl> impl_;
};

// todo: review
class SerialException : public std::exception
{
  // Disable copy constructors
  SerialException& operator=(const SerialException&);
  std::string e_what_;

public:
  explicit SerialException(const char* description)
  {
    std::stringstream ss;
    ss << "SerialException " << description << " failed.";
    e_what_ = ss.str();
  }
  SerialException(const SerialException& other) : e_what_(other.e_what_)
  {
  }
  virtual ~SerialException() throw()
  {
  }
  virtual const char* what() const throw()
  {
    return e_what_.c_str();
  }
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER_VESC_INTERFACE_H_
