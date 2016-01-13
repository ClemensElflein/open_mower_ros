// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_INTERFACE_H_
#define VESC_DRIVER_VESC_INTERFACE_H_

#include <string>
#include <sstream>
#include <exception>
#include <stdexcept>

#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

/**
 * Class providing an interface to the Vedder VESC motor controller via a serial port interface.
 */
class VescInterface : private boost::noncopyable
{
public:

  typedef boost::function<void (const VescPacketConstPtr&)> PacketHandlerFunction;
  typedef boost::function<void (const std::string&)> ErrorHandlerFunction;

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
  boost::scoped_ptr<Impl> impl_;
};

// todo: review
class SerialException : public std::exception
{
  // Disable copy constructors
  SerialException& operator=(const SerialException&);
  std::string e_what_;
public:
  SerialException (const char *description) {
      std::stringstream ss;
      ss << "SerialException " << description << " failed.";
      e_what_ = ss.str();
  }
  SerialException (const SerialException& other) : e_what_(other.e_what_) {}
  virtual ~SerialException() throw() {}
  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_INTERFACE_H_
