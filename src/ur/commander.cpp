#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/log.h"

static const int32_t MULT_JOINTSTATE_ = 1000000;
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string SERVO_J_REPLACE("{{SERVO_J_REPLACE}}");
static const std::string SPEED_J_REPLACE("{{SPEED_J_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string POSITION_PROGRAM = R"(
def driverProg():
	MULT_jointstate = {{JOINT_STATE_REPLACE}}

	SERVO_IDLE = 0
	SERVO_RUNNING = 1
	cmd_servo_state = SERVO_IDLE
	cmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	def set_servo_setpoint(q):
		enter_critical
		cmd_servo_state = SERVO_RUNNING
		cmd_servo_q = q
		exit_critical
	end

	thread servoThread():
		state = SERVO_IDLE
		while True:
			enter_critical
			q = cmd_servo_q
			do_brake = False
			if (state == SERVO_RUNNING) and (cmd_servo_state == SERVO_IDLE):
				do_brake = True
			end
			state = cmd_servo_state
			cmd_servo_state = SERVO_IDLE
			exit_critical
			if do_brake:
				stopj(1.0)
				sync()
			elif state == SERVO_RUNNING:
				servoj(q, {{SERVO_J_REPLACE}})
			else:
				sync()
			end
		end
	end

  socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  thread_servo = run servoThread()
  keepalive = 1
  while keepalive > 0:
	  params_mult = socket_read_binary_integer(6+1)
	  if params_mult[0] > 0:
		  q = [params_mult[1] / MULT_jointstate, params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate]
		  keepalive = params_mult[7]
		  set_servo_setpoint(q)
	  end
  end
  sleep(.1)
  socket_close()
  kill thread_servo
end
)";
static const std::string VELOCITY_PROGRAM = R"(
def driverProg():
	MULT_jointstate = {{JOINT_STATE_REPLACE}}

	SPEED_IDLE = 0
	SPEED_RUNNING = 1
	cmd_speed_state = SPEED_IDLE
	cmd_speed_qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  cmd_speed_qdd = 0.0

	def set_speed_setpoint(qd, qdd):
		enter_critical
		cmd_speed_state = SPEED_RUNNING
		cmd_speed_qd = qd
    cmd_speed_qdd = qdd
		exit_critical
	end

	thread speedThread():
		state = SPEED_IDLE
		while True:
			enter_critical
			qd = cmd_speed_qd
      qdd = cmd_speed_qdd
			do_brake = False
			if (state == SPEED_RUNNING) and (cmd_speed_state == SPEED_IDLE):
				do_brake = True
			end
			state = cmd_speed_state
			cmd_speed_state = SPEED_IDLE
			exit_critical
			if do_brake:
				stopj(1.0)
				sync()
			elif state == SPEED_RUNNING:
				speedj(qd, qdd{{SPEED_J_REPLACE}})
			else:
				sync()
			end
		end
	end

  socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  thread_speed = run speedThread()
  keepalive = 1
  while keepalive > 0:
	  params_mult = socket_read_binary_integer(6+2)
	  if params_mult[0] > 0:
		  qd = [params_mult[1] / MULT_jointstate, params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate]
      qdd = params_mult[7] / MULT_jointstate
		  keepalive = params_mult[8]
		  set_speed_setpoint(qd, qdd)
	  end
  end
  sleep(.1)
  socket_close()
  kill thread_speed
end
)";

URCommander::URCommander(URStream &stream, const URCommanderOpts &options)
  : stream_(stream), server_(options.reverse_port), servo_loop_running_(false), servoj_time_(options.servoj_time)
{
  preparePositionProgram(options);

  if (!server_.bind())
  {
    LOG_ERROR("Failed to bind server, the port %d is likely already in use", options.reverse_port);
    std::exit(-1);
  }
}

void URCommander::preparePositionProgram(const URCommanderOpts &options)
{
  std::string res(POSITION_PROGRAM);
  res.replace(res.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));

  std::ostringstream out;
  out << "t=" << std::fixed << std::setprecision(4) << options.servoj_time;
  if (options.version_3)
    out << ", lookahead_time=" << options.servoj_lookahead_time << ", gain=" << options.servoj_gain;

  res.replace(res.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), out.str());
  res.replace(res.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), options.reverse_ip);
  res.replace(res.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(options.reverse_port));
  position_program_ = res;
}

void URCommander::prepareVelocityProgram(const URCommanderOpts &options, const std::string &speedj_replace)
{
  std::string res(VELOCITY_PROGRAM);
  res.replace(res.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));

  res.replace(res.find(SPEED_J_REPLACE), SPEED_J_REPLACE.length(), speedj_replace);
  res.replace(res.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), options.reverse_ip);
  res.replace(res.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(options.reverse_port));
  velocity_program_ = res;
}

bool URCommander::startSpeedLoop()
{
  if (servo_loop_running_)
    return true;

  LOG_INFO("Uploading trajectory program to robot");

  if (!uploadProg(velocity_program_))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }

  LOG_DEBUG("Awaiting incoming robot connection");

  if (!server_.accept())
  {
    LOG_ERROR("Failed to accept incoming robot connection");
    return false;
  }

  return (servo_loop_running_ = true);
}

void URCommander::stopSpeedLoop()
{
  if (!servo_loop_running_)
    return;

  server_.disconnectClient();

  servo_loop_running_ = false;
}

bool URCommander::startServoLoop()
{
  if (servo_loop_running_)
    return true;

  LOG_INFO("Uploading trajectory program to robot");

  if (!uploadProg(position_program_))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }

  LOG_DEBUG("Awaiting incoming robot connection");

  if (!server_.accept())
  {
    LOG_ERROR("Failed to accept incoming robot connection");
    return false;
  }

  return (servo_loop_running_ = true);
}

void URCommander::stopServoLoop()
{
  if (!servo_loop_running_)
    return;

  server_.disconnectClient();

  servo_loop_running_ = false;
}

bool URCommander::write(const std::string &s)
{
  if (servo_loop_running_)
  {
    LOG_WARN("Sending a new command while servo loop is running would interfere. Ignoring request");
    return false;
  }

  size_t len = s.size();
  const uint8_t *data = reinterpret_cast<const uint8_t *>(s.c_str());
  size_t written;
  return stream_.write(data, len, written);
}

void URCommander::formatArray(std::ostringstream &out, std::array<double, 6> &values)
{
  std::string mod("[");
  for (auto const &val : values)
  {
    out << mod << val;
    mod = ",";
  }
  out << "]";
}

bool URCommander::servoj(std::array<double, 6> &positions, bool keep_alive)
{
  if (!servo_loop_running_)
    return false;

  uint8_t buf[sizeof(uint32_t) * 7];
  uint8_t *idx = buf;

  for (auto const &pos : positions)
  {
    int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE_);
    val = htobe32(val);
    idx += append(idx, val);
  }

  int32_t val = htobe32(static_cast<int32_t>(keep_alive));
  append(idx, val);

  size_t written;
  return server_.write(buf, sizeof(buf), written);
}

bool URCommander::speedj(std::array<double, 6> &speeds, double acceleration, bool keep_alive)
{
  if (!servo_loop_running_)
    return false;

  uint8_t buf[sizeof(uint32_t) * 8];
  uint8_t *idx = buf;

  for (auto const &spe : speeds)
  {
    int32_t val = static_cast<int32_t>(spe * MULT_JOINTSTATE_);
    val = htobe32(val);
    idx += append(idx, val);
  }

  int32_t val = htobe32(static_cast<int32_t>(acceleration * MULT_JOINTSTATE_));
  idx += append(idx, val);

  val = htobe32(static_cast<int32_t>(keep_alive));
  append(idx, val);

  size_t written;
  return server_.write(buf, sizeof(buf), written);
}

bool URCommander::uploadProg(const std::string &s)
{
  LOG_DEBUG("Sending program [%s]", s.c_str());
  return write(s);
}

bool URCommander::setToolVoltage(uint8_t voltage)
{
  if (voltage != 0 || voltage != 12 || voltage != 24)
    return false;

  std::ostringstream out;
  out << "set_tool_voltage(" << (int)voltage << ")\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander::setFlag(uint8_t pin, bool value)
{
  std::ostringstream out;
  out << "set_flag(" << (int)pin << "," << (value ? "True" : "False") << ")\n";
  std::string s(out.str());
  return write(s);
}
bool URCommander::setPayload(double value)
{
  std::ostringstream out;
  out << "set_payload(" << std::fixed << std::setprecision(5) << value << ")\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander::stopj(double a)
{
  std::ostringstream out;
  out << "stopj(" << std::fixed << std::setprecision(5) << a << ")\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V1_X::setAnalogOut(uint8_t pin, double value)
{
  std::ostringstream out;
  out << "sec io_fun():\n"
      << "set_analog_out(" << (int)pin << "," << std::fixed << std::setprecision(4) << value << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V1_X::setDigitalOut(uint8_t pin, bool value)
{
  std::ostringstream out;
  out << "sec io_fun():\n"
      << "set_digital_out(" << (int)pin << "," << (value ? "True" : "False") << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V3_X::setAnalogOut(uint8_t pin, double value)
{
  std::ostringstream out;
  out << "sec io_fun():\n"
      << "set_standard_analog_out(" << (int)pin << "," << std::fixed << std::setprecision(5) << value << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_V3_X::setDigitalOut(uint8_t pin, bool value)
{
  std::ostringstream out;
  std::string func;

  if (pin < 8)
  {
    func = "set_standard_digital_out";
  }
  else if (pin < 16)
  {
    func = "set_configurable_digital_out";
    pin -= 8;
  }
  else if (pin < 18)
  {
    func = "set_tool_digital_out";
    pin -= 16;
  }
  else
    return false;

  out << "sec io_fun():\n"
      << func << "(" << (int)pin << "," << (value ? "True" : "False") << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_e::setAnalogOut(uint8_t pin, double value)
{
  std::ostringstream out;
  out << "sec io_fun():\n"
      << "set_standard_analog_out(" << (int)pin << "," << std::fixed << std::setprecision(5) << value << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

bool URCommander_e::setDigitalOut(uint8_t pin, bool value)
{
  std::ostringstream out;
  std::string func;

  if (pin < 8)
  {
    func = "set_standard_digital_out";
  }
  else if (pin < 16)
  {
    func = "set_configurable_digital_out";
    pin -= 8;
  }
  else if (pin < 18)
  {
    func = "set_tool_digital_out";
    pin -= 16;
  }
  else
    return false;

  out << "sec io_fun():\n"
      << func << "(" << (int)pin << "," << (value ? "True" : "False") << ")\n"
      << "end\n";
  std::string s(out.str());
  return write(s);
}

URCommander_CB::URCommander_CB(URStream &stream, const URCommanderOpts &options) : URCommander(stream, options)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(5);
  out << ", " << 0.008;

  prepareVelocityProgram(options, out.str());
}

URCommander_e::URCommander_e(URStream &stream, const URCommanderOpts &options) : URCommander(stream, options)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(5);
  out << ", " << 0.002;

  prepareVelocityProgram(options, out.str());
}
