#pragma once
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include "ur_modern_driver/ur/server.h"
#include "ur_modern_driver/ur/stream.h"

struct URCommanderOpts
{
  std::string reverse_ip;
  int reverse_port;
  bool version_3;
  double servoj_time{ 0.008 };
  double servoj_lookahead_time{ 0.03 };
  double servoj_gain{ 300. };
};

class URCommander
{
private:
  URStream &stream_;

  URServer server_;

  std::string program_;
  double servoj_time_, servoj_lookahead_time_, servoj_gain_;
  bool servo_loop_running_;

  template <typename T>
  size_t append(uint8_t *buffer, T &val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

protected:
  bool write(const std::string &s);
  void formatArray(std::ostringstream &out, std::array<double, 6> &values);

public:
  URCommander(URStream &stream, const URCommanderOpts &options);

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration) = 0;
  virtual bool setDigitalOut(uint8_t pin, bool value) = 0;
  virtual bool setAnalogOut(uint8_t pin, double value) = 0;

  // shared
  bool uploadProg(const std::string &s);
  bool stopj(double a = 10.0);
  bool setToolVoltage(uint8_t voltage);
  bool setFlag(uint8_t pin, bool value);
  bool setPayload(double value);

  bool startServoLoop();
  void stopServoLoop();
  bool servoj(std::array<double, 6> &positions, bool keep_alive);
  double servojTime()
  {
    return servoj_time_;
  }
};

class URCommander_V1_X : public URCommander
{
public:
  URCommander_V1_X(URStream &stream, const URCommanderOpts &options) : URCommander(stream, options)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration);
  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
};

class URCommander_V3_X : public URCommander
{
public:
  URCommander_V3_X(URStream &stream, const URCommanderOpts &options) : URCommander(stream, options)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration) = 0;
  virtual bool setDigitalOut(uint8_t pin, bool value);
  virtual bool setAnalogOut(uint8_t pin, double value);
};

class URCommander_V3_1__2 : public URCommander_V3_X
{
public:
  URCommander_V3_1__2(URStream &stream, const URCommanderOpts &options) : URCommander_V3_X(stream, options)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration);
};

class URCommander_V3_3 : public URCommander_V3_X
{
public:
  URCommander_V3_3(URStream &stream, const URCommanderOpts &options) : URCommander_V3_X(stream, options)
  {
  }

  virtual bool speedj(std::array<double, 6> &speeds, double acceleration);
};
