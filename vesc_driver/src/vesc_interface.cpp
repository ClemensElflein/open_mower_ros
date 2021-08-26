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

#include "vesc_driver/vesc_interface.h"

namespace vesc_driver
{
class VescInterface::Impl
{
public:
  Impl()
    : serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100), serial::eightbits, serial::parity_none,
              serial::stopbits_one, serial::flowcontrol_none)
  {
  }

  void* rxThread(void);

  static void* rxThreadHelper(void* context)
  {
    return ((VescInterface::Impl*)context)->rxThread();
  }

  pthread_t rx_thread_;
  bool rx_thread_run_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  serial::Serial serial_;
  VescFrame::CRC send_crc_;
};

void* VescInterface::Impl::rxThread(void)
{
  Buffer buffer;
  buffer.reserve(4096);

  while (rx_thread_run_)
  {
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer.empty())
    {
      // search buffer for valid packet(s)
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while (iter != buffer.end())
      {
        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter || VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
        {
          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet = VescPacketFactory::createPacket(iter, buffer.end(), &bytes_needed, &error);
          if (packet)
          {
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0)
            {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
                 << std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->getFrame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          }
          else if (bytes_needed > 0)
          {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          }
          else
          {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end())
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0)
      {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }

    // attempt to read at least bytes_needed bytes from the serial port
    int bytes_to_read = std::max(bytes_needed, std::min(4096, static_cast<int>(serial_.available())));
    int bytes_read = serial_.read(buffer, bytes_to_read);
    if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty())
    {
      error_handler_("Possibly out-of-sync with VESC, read timout in the middle of a frame.");
    }
  }
}

VescInterface::VescInterface(const std::string& port, const PacketHandlerFunction& packet_handler,
                             const ErrorHandlerFunction& error_handler)
  : impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty())
    connect(port);
}

VescInterface::~VescInterface()
{
  // stops the motor
  setDutyCycle(0.0);

  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string& port)
{
  // todo - mutex?

  if (isConnected())
  {
    throw SerialException("Already connected to serial port.");
  }

  // connect to serial port
  try
  {
    impl_->serial_.setPort(port);
    impl_->serial_.open();
  }
  catch (const std::exception& e)
  {
    std::stringstream ss;
    ss << "Failed to open the serial port to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  impl_->rx_thread_run_ = true;
  int result = pthread_create(&impl_->rx_thread_, NULL, &VescInterface::Impl::rxThreadHelper, impl_.get());
  assert(0 == result);
}

void VescInterface::disconnect()
{
  // todo - mutex?

  if (isConnected())
  {
    // bring down read thread
    impl_->rx_thread_run_ = false;
    int result = pthread_join(impl_->rx_thread_, NULL);
    assert(0 == result);

    impl_->serial_.close();
  }
}

bool VescInterface::isConnected() const
{
  return impl_->serial_.isOpen();
}

void VescInterface::send(const VescPacket& packet)
{
  size_t written = impl_->serial_.write(packet.getFrame());
  if (written != packet.getFrame().size())
  {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << packet.getFrame().size() << ".";
    throw SerialException(ss.str().c_str());
  }
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetVelocityERPM(speed));
}

void VescInterface::setPosition(double position)
{
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)
{
  send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver
