#ifndef DIFFDRIVE_ESP32_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ESP32_ARDUINO_COMMS_HPP

#ifndef CPPHTTPLIB_OPENSSL_SUPPORT
#define CPPHTTPLIB_OPENSSL_SUPPORT
#endif
#include "httplib.h"

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class Esp32Comms
{

public:
  Esp32Comms() = default;

  void connect(const std::string &ip, int32_t timeout_ms)
  {
    cli = std::make_shared<httplib::Client>(ip);
    timeout_ms_ = timeout_ms;
  }

  void disconnect()
  {
    cli->stop();
    // serial_conn_.Close();
  }

  bool connected() const
  {
    auto res = cli->Get("/control?var=e&val=0_0");

    return res->status == 200;
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = true)
  {

    httplib::Result response;
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      response = cli->Get(msg_to_send);

      if (print_output)
      {
        std::cout << "Sent: " << msg_to_send << " Recv: " << response->body << std::endl;
      }
    }
    catch (...)
    {
      std::cerr << "The ReadByte() call has timed out." << std::endl;
    }

    if (response->status == 200)
    {
      return response->body;
    }
    return "0_0"; // default
  }

  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    try
    {
      std::string response = send_msg("/control?var=e&val=0_0");

      std::string delimiter = " ";
      size_t del_pos = response.find(delimiter);
      std::string token_1 = response.substr(0, del_pos);
      std::string token_2 = response.substr(del_pos + delimiter.length());

      val_1 = std::atoi(token_1.c_str());
      val_2 = std::atoi(token_2.c_str());
    }
    catch (...)
    {
      val_1 = 0;
      val_2 = 0;
    }
  }
  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    try
    {
      ss << "/control?var=o&val=" << val_1 * 2 << "_" << val_2 * 2;
      send_msg((ss.str()).c_str());
    }
    catch (...)
    {
    }
    // ss << "m " << val_1 << " " << val_2 << "\r";
    // send_msg((ss.str()).c_str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
  std::shared_ptr<httplib::Client> cli;
  int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP