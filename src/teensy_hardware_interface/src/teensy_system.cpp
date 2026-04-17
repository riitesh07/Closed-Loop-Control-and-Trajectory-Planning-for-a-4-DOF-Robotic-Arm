#include "teensy_hardware_interface/teensy_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <cstdio>
#include <cmath>

#include <string>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <cctype>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace teensy_hardware_interface
{

static rclcpp::Clock g_clock(RCL_STEADY_TIME);

// Per-instance RX accumulation without changing your header
static std::unordered_map<const TeensySystem*, std::string> g_rx_accum;
static std::unordered_map<const TeensySystem*, std::string> g_last_rx_line;

// Per-instance "last valid HW command sent" (NaN/Inf protection) WITHOUT header change
static std::unordered_map<const TeensySystem*, std::vector<double>> g_last_hw_sent;

// Per-instance: have we received at least one valid Q state yet?
static std::unordered_map<const TeensySystem*, bool> g_seen_first_state;

// Per-instance: did we already sync commands_ to positions_ once (after first state)?
static std::unordered_map<const TeensySystem*, bool> g_synced_cmd_to_state;

// Per-instance: keep re-sending AUTO until we see the first Q
static std::unordered_map<const TeensySystem*, std::chrono::steady_clock::time_point> g_next_auto_send;
static constexpr std::chrono::milliseconds AUTO_RESEND_PERIOD{300};
static constexpr std::chrono::milliseconds AUTO_LOG_PERIOD{2000};
static std::unordered_map<const TeensySystem*, std::chrono::steady_clock::time_point> g_next_auto_log;

// Per-instance: per-instance log counters
static std::unordered_map<const TeensySystem*, int> g_rx_log_counter;
static std::unordered_map<const TeensySystem*, int> g_tx_log_counter;

// Per-instance: pending TX buffer (handles non-blocking partial writes safely)
static std::unordered_map<const TeensySystem*, std::string> g_tx_pending;
static constexpr size_t MAX_TX_PENDING_BYTES = 4096;

// Separate STATE mapping (read) from COMMAND mapping (write)
static std::unordered_map<const TeensySystem*, std::vector<double>> g_state_sign;
static std::unordered_map<const TeensySystem*, std::vector<double>> g_state_scale;
static std::unordered_map<const TeensySystem*, std::vector<double>> g_state_offset;

// Additional bias (optional)
//   - read():  ROS = base_state - bias
//   - write(): HW  = base_cmd + bias
static std::unordered_map<const TeensySystem*, std::vector<double>> g_bias;

// --------- STARTUP HOLD ----------
static std::unordered_map<const TeensySystem*, std::chrono::steady_clock::time_point> g_hold_until;
static std::unordered_map<const TeensySystem*, std::chrono::milliseconds> g_startup_hold_ms;
static std::unordered_map<const TeensySystem*, bool> g_startup_hold_strict;
static constexpr double STARTUP_JUMP_RAD = 0.10;  // used only if strict=0
// --------------------------------

// Passthrough mode (Teensy I/O already ROS radians)
static std::unordered_map<const TeensySystem*, bool> g_io_is_ros;

static bool get_double_param(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  double & out_val)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return false;
  try { out_val = std::stod(it->second); return true; }
  catch (...) { return false; }
}

static bool get_int_param(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  int & out_val)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return false;
  try { out_val = std::stoi(it->second); return true; }
  catch (...) { return false; }
}

static inline void trim_inplace(std::string & s)
{
  auto not_space = [](unsigned char c){ return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
}

static bool flush_pending(int fd, std::string & pending)
{
  while (!pending.empty()) {
    errno = 0;
    const ssize_t w = ::write(fd, pending.data(), pending.size());
    if (w > 0) {
      pending.erase(0, static_cast<size_t>(w));
      continue;
    }
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }
    return false;
  }
  return true;
}

// accepts lines like:
//   "Q a b c d"
//   "OK Q a b c d"
//   "Q: a b c d"
static bool parse_Q_line(
  const std::string & line,
  double & cont,
  double & rev1,
  double & rev2,
  double & wrist)
{
  if (line.empty()) return false;

  size_t p = line.find('Q');
  if (p == std::string::npos) p = line.find('q');
  if (p == std::string::npos) return false;

  const char* s = line.c_str() + p;

  if (s[0] == 'q') {
    int n = std::sscanf(s, "q %lf %lf %lf %lf", &cont, &rev1, &rev2, &wrist);
    if (n == 4) return true;
    n = std::sscanf(s, "q: %lf %lf %lf %lf", &cont, &rev1, &rev2, &wrist);
    if (n == 4) return true;
    n = std::sscanf(s, "q, %lf %lf %lf %lf", &cont, &rev1, &rev2, &wrist);
    return (n == 4);
  } else {
    int n = std::sscanf(s, "Q %lf %lf %lf %lf", &cont, &rev1, &rev2, &wrist);
    if (n == 4) return true;
    n = std::sscanf(s, "Q: %lf %lf %lf %lf", &cont, &rev1, &rev2, &wrist);
    if (n == 4) return true;
    n = std::sscanf(s, "Q, %lf %lf %lf %lf", &cont, &rev1, &rev2, &wrist);
    return (n == 4);
  }
}

hardware_interface::CallbackReturn TeensySystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  const size_t n = info_.joints.size();
  positions_.assign(n, 0.0);
  velocities_.assign(n, 0.0);
  commands_.assign(n, 0.0);

  sign_.assign(n, 1.0);
  scale_.assign(n, 1.0);
  offset_.assign(n, 0.0);

  g_last_hw_sent[this].assign(n, 0.0);

  g_state_sign[this].assign(n, 1.0);
  g_state_scale[this].assign(n, 1.0);
  g_state_offset[this].assign(n, 0.0);

  g_bias[this].assign(n, 0.0);

  // io_is_ros default: RAW mode unless explicitly set otherwise
  {
    int tmp = 0; // RAW default
    if (get_int_param(info_, "io_is_ros", tmp)) {
      g_io_is_ros[this] = (tmp != 0);
    } else {
      g_io_is_ros[this] = false;
    }
  }

  // NEW: startup hold params
  {
    int ms = 8000;
    (void)get_int_param(info_, "startup_hold_ms", ms);
    if (ms < 0) ms = 0;
    g_startup_hold_ms[this] = std::chrono::milliseconds(ms);

    int strict = 1;
    (void)get_int_param(info_, "startup_hold_strict", strict);
    g_startup_hold_strict[this] = (strict != 0);
  }

  // Serial params
  auto it_port = info_.hardware_parameters.find("port");
  port_ = (it_port != info_.hardware_parameters.end()) ? it_port->second : "/dev/ttyACM0";

  auto it_baud = info_.hardware_parameters.find("baudrate");
  if (it_baud != info_.hardware_parameters.end()) {
    try { baudrate_ = std::stoi(it_baud->second); }
    catch (...) { baudrate_ = 115200; }
  } else {
    baudrate_ = 115200;
  }

  int tmp = 0;
  if (get_int_param(info_, "send_auto_on_activate", tmp)) {
    send_auto_on_activate_ = (tmp != 0);
  }
  if (get_int_param(info_, "log_div", tmp)) {
    if (tmp > 0) log_div_ = tmp;
  }

  idx_base_  = -1;
  idx_arm1_  = -1;
  idx_arm2_  = -1;
  idx_wrist_ = -1;

  // Track which params were actually provided (so we don't "guess" wrong)
  std::vector<bool> cmd_sign_set(n,false), cmd_scale_set(n,false), cmd_offset_set(n,false);
  std::vector<bool> st_sign_set(n,false),  st_scale_set(n,false),  st_offset_set(n,false);
  std::vector<bool> bias_set(n,false);

  for (size_t i = 0; i < n; ++i) {
    const std::string & jn = info_.joints[i].name;

    if (jn == "continuous") idx_base_ = static_cast<int>(i);
    else if (jn == "revolute1") idx_arm1_ = static_cast<int>(i);
    else if (jn == "revolute2") idx_arm2_ = static_cast<int>(i);
    else if (jn == "revolute3") idx_wrist_ = static_cast<int>(i);

    double v = 0.0;

    // COMMAND mapping params:
    if (get_double_param(info_, "sign_" + jn, v)) {
      sign_[i] = (v >= 0.0) ? 1.0 : -1.0;
      cmd_sign_set[i] = true;
    }
    if (get_double_param(info_, "scale_" + jn, v)) {
      if (std::isfinite(v) && v != 0.0) {
        scale_[i] = v;
        cmd_scale_set[i] = true;
      }
    }
    if (get_double_param(info_, "offset_" + jn, v)) {
      if (std::isfinite(v)) {
        offset_[i] = v;
        cmd_offset_set[i] = true;
      }
    }

    // STATE mapping params:
    if (get_double_param(info_, "sign_state_" + jn, v)) {
      g_state_sign[this][i] = (v >= 0.0) ? 1.0 : -1.0;
      st_sign_set[i] = true;
    }
    if (get_double_param(info_, "scale_state_" + jn, v)) {
      if (std::isfinite(v) && v != 0.0) {
        g_state_scale[this][i] = v;
        st_scale_set[i] = true;
      }
    }
    if (get_double_param(info_, "offset_state_" + jn, v)) {
      if (std::isfinite(v)) {
        g_state_offset[this][i] = v;
        st_offset_set[i] = true;
      }
    }

    // bias_<jointname> (radians)
    if (get_double_param(info_, "bias_" + jn, v)) {
      if (std::isfinite(v)) {
        g_bias[this][i] = v;
        bias_set[i] = true;
      }
    }
  }

  // Symmetry rules:
  for (size_t i = 0; i < n; ++i) {
    if (!cmd_sign_set[i] && st_sign_set[i]) sign_[i] = g_state_sign[this][i];
    if (!st_sign_set[i] && cmd_sign_set[i]) g_state_sign[this][i] = sign_[i];

    if (!cmd_scale_set[i] && st_scale_set[i]) scale_[i] = g_state_scale[this][i];
    if (!st_scale_set[i] && cmd_scale_set[i]) g_state_scale[this][i] = scale_[i];

    if (!cmd_offset_set[i] && st_offset_set[i]) offset_[i] = g_state_offset[this][i];
    if (!st_offset_set[i] && cmd_offset_set[i]) g_state_offset[this][i] = offset_[i];
  }

  g_rx_accum[this].clear();
  g_last_rx_line[this].clear();
  g_seen_first_state[this] = false;
  g_synced_cmd_to_state[this] = false;
  g_next_auto_send[this] = std::chrono::steady_clock::now();
  g_next_auto_log[this]  = std::chrono::steady_clock::now();

  g_rx_log_counter[this] = 0;
  g_tx_log_counter[this] = 0;
  g_tx_pending[this].clear();

  g_hold_until[this] = std::chrono::steady_clock::time_point{}; // not holding yet

  RCLCPP_INFO(
    rclcpp::get_logger("TeensySystem"),
    "Initialized TeensySystem with %zu joints, port='%s', baudrate=%d, io_is_ros=%d",
    n, port_.c_str(), baudrate_, g_io_is_ros[this] ? 1 : 0);

  RCLCPP_INFO(
    rclcpp::get_logger("TeensySystem"),
    "Joint indices: base(continuous)=%d arm1(revolute1)=%d arm2(revolute2)=%d wrist(revolute3)=%d",
    idx_base_, idx_arm1_, idx_arm2_, idx_wrist_);

  for (size_t i = 0; i < n; ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("TeensySystem"),
      "joint[%zu]='%s' CMD{sign=%.0f scale=%.6f offset=%.6f}  STATE{sign=%.0f scale=%.6f offset=%.6f}  BIAS{%.6f}",
      i, info_.joints[i].name.c_str(),
      sign_[i], scale_[i], offset_[i],
      g_state_sign[this][i], g_state_scale[this][i], g_state_offset[this][i],
      g_bias[this][i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("TeensySystem"),
    "Startup hold: %ld ms, strict=%d",
    (long)g_startup_hold_ms[this].count(),
    g_startup_hold_strict[this] ? 1 : 0);

  if (g_io_is_ros[this]) {
    RCLCPP_WARN(rclcpp::get_logger("TeensySystem"),
      "io_is_ros=1: mapping/offset/bias params are ignored. "
      "For raw encoder mode, set <param name=\"io_is_ros\">0</param>.");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

auto TeensySystem::export_state_interfaces()
  -> std::vector<hardware_interface::StateInterface>
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
  }
  return interfaces;
}

auto TeensySystem::export_command_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &commands_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn TeensySystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("TeensySystem"),
    "Activating TeensySystem – opening serial port '%s' at %d baud",
    port_.c_str(), baudrate_);

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
      "Failed to open '%s': %s", port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
      "tcgetattr failed on '%s': %s", port_.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfmakeraw(&tty);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  speed_t speed = B115200;
  switch (baudrate_) {
    case 9600:   speed = B9600; break;
    case 19200:  speed = B19200; break;
    case 38400:  speed = B38400; break;
    case 57600:  speed = B57600; break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("TeensySystem"),
        "Unsupported baudrate %d, falling back to 115200", baudrate_);
      speed = B115200;
      break;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
      "tcsetattr failed on '%s': %s", port_.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  tcflush(fd_, TCIOFLUSH);

  g_rx_accum[this].clear();
  g_last_rx_line[this].clear();

  g_last_hw_sent[this].resize(info_.joints.size(), 0.0);

  g_seen_first_state[this] = false;
  g_synced_cmd_to_state[this] = false;
  g_next_auto_send[this] = std::chrono::steady_clock::now();
  g_next_auto_log[this]  = std::chrono::steady_clock::now();

  g_rx_log_counter[this] = 0;
  g_tx_log_counter[this] = 0;
  g_tx_pending[this].clear();

  g_hold_until[this] = std::chrono::steady_clock::time_point{}; // not holding yet

  RCLCPP_INFO(rclcpp::get_logger("TeensySystem"),
    "Serial port '%s' opened successfully", port_.c_str());

  if (send_auto_on_activate_) {
    auto & pending = g_tx_pending[this];
    pending.append("AUTO\r\n");
    (void)flush_pending(fd_, pending);
    RCLCPP_INFO(rclcpp::get_logger("TeensySystem"), "Queued: AUTO (initial)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TeensySystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("TeensySystem"),
    "Deactivating TeensySystem – closing serial port");

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  g_rx_accum.erase(this);
  g_last_rx_line.erase(this);
  g_last_hw_sent.erase(this);
  g_seen_first_state.erase(this);
  g_synced_cmd_to_state.erase(this);
  g_next_auto_send.erase(this);
  g_next_auto_log.erase(this);
  g_rx_log_counter.erase(this);
  g_tx_log_counter.erase(this);
  g_tx_pending.erase(this);

  g_state_sign.erase(this);
  g_state_scale.erase(this);
  g_state_offset.erase(this);
  g_bias.erase(this);

  g_hold_until.erase(this);
  g_startup_hold_ms.erase(this);
  g_startup_hold_strict.erase(this);
  g_io_is_ros.erase(this);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TeensySystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
      "read() called but serial fd_ < 0 (port not open)");
    return hardware_interface::return_type::ERROR;
  }

  // 1) Pull all available bytes (non-blocking)
  char tmp[256];
  for (;;) {
    errno = 0;
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0) {
      auto & acc = g_rx_accum[this];
      acc.append(tmp, tmp + n);

      if (acc.size() > 8192) {
        acc.erase(0, acc.size() - 4096);
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("TeensySystem"),
          g_clock, 2000,
          "RX buffer grew too large; trimming to last 4096 bytes (missing newlines?).");
      }
      continue;
    }
    if (n == 0) break;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;

    int err = errno;
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
      "Serial read error (errno=%d: %s). Closing fd_.",
      err, std::strerror(err));
    ::close(fd_);
    fd_ = -1;
    return hardware_interface::return_type::ERROR;
  }

  // 2) Consume complete lines
  bool got_state = false;
  double cont_hw=0, rev1_hw=0, rev2_hw=0, wrist_hw=0;

  auto & acc = g_rx_accum[this];

  for (;;) {
    auto pos = acc.find_first_of("\r\n");
    if (pos == std::string::npos) break;

    std::string line = acc.substr(0, pos);

    size_t erase_len = pos;
    while (erase_len < acc.size() && (acc[erase_len] == '\r' || acc[erase_len] == '\n')) {
      ++erase_len;
    }
    acc.erase(0, erase_len);

    trim_inplace(line);
    if (line.empty()) continue;

    g_last_rx_line[this] = line;

    int & rx_ctr = g_rx_log_counter[this];
    if ((rx_ctr++ % log_div_) == 0) {
      RCLCPP_INFO(rclcpp::get_logger("TeensySystem"),
        "RX from Teensy: %s", line.c_str());
    }

    double c=0, r1=0, r2=0, w=0;
    if (parse_Q_line(line, c, r1, r2, w)) {
      got_state = true;
      cont_hw  = c;
      rev1_hw  = r1;
      rev2_hw  = r2;
      wrist_hw = w;
    }
  }

  auto to_ros_state = [&](int idx, double hw) -> double {
    if (g_io_is_ros[this]) {
      return hw;
    }
    const size_t u = static_cast<size_t>(idx);
    const double denom = g_state_scale[this][u] * g_state_sign[this][u];
    if (!std::isfinite(denom) || denom == 0.0) return 0.0;
    const double base = (hw / denom) - g_state_offset[this][u];
    return base - g_bias[this][u];
  };

  const double dt = period.seconds();
  const double inv_dt = (dt > 1e-9) ? (1.0 / dt) : 0.0;

  if (got_state) {
    if (idx_arm1_ < 0 || idx_base_ < 0 || idx_arm2_ < 0 || idx_wrist_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
        "Joint name mapping failed in read(). Need joints: continuous, revolute1, revolute2, revolute3");
      return hardware_interface::return_type::ERROR;
    }

    const double cont_ros  = to_ros_state(idx_base_,  cont_hw);
    const double rev1_ros  = to_ros_state(idx_arm1_,  rev1_hw);
    const double rev2_ros  = to_ros_state(idx_arm2_,  rev2_hw);
    const double wrist_ros = to_ros_state(idx_wrist_, wrist_hw);

    auto update_joint = [&](int idx, double new_pos) {
      const size_t u = static_cast<size_t>(idx);
      const double old = positions_[u];
      positions_[u]  = new_pos;
      velocities_[u] = (inv_dt > 0.0) ? ((new_pos - old) * inv_dt) : 0.0;
    };

    update_joint(idx_base_,  cont_ros);
    update_joint(idx_arm1_,  rev1_ros);
    update_joint(idx_arm2_,  rev2_ros);
    update_joint(idx_wrist_, wrist_ros);

    g_seen_first_state[this] = true;

    if (!g_synced_cmd_to_state[this]) {
      for (size_t i = 0; i < commands_.size(); ++i) {
        commands_[i] = positions_[i];
      }
      g_synced_cmd_to_state[this] = true;

      // Enable startup hold
      g_hold_until[this] = std::chrono::steady_clock::now() + g_startup_hold_ms[this];

      RCLCPP_WARN(rclcpp::get_logger("TeensySystem"),
        "First Q received -> synced commands to state. Startup hold enabled for %ld ms (strict=%d).",
        (long)g_startup_hold_ms[this].count(),
        g_startup_hold_strict[this] ? 1 : 0);
    }
  } else {
    for (size_t i = 0; i < velocities_.size(); ++i) velocities_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TeensySystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
      "write() called but serial fd_ < 0 (port not open)");
    return hardware_interface::return_type::ERROR;
  }

  if (idx_arm1_ < 0 || idx_base_ < 0 || idx_arm2_ < 0 || idx_wrist_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"),
      "Joint name mapping failed. Need joints: continuous, revolute1, revolute2, revolute3");
    return hardware_interface::return_type::ERROR;
  }

  // If we haven't seen Q yet, keep trying AUTO
  if (!g_seen_first_state[this]) {
    if (send_auto_on_activate_) {
      auto now = std::chrono::steady_clock::now();
      if (now >= g_next_auto_send[this]) {
        auto & pending = g_tx_pending[this];
        pending.append("AUTO\r\n");
        (void)flush_pending(fd_, pending);
        g_next_auto_send[this] = now + AUTO_RESEND_PERIOD;

        if (now >= g_next_auto_log[this]) {
          g_next_auto_log[this] = now + AUTO_LOG_PERIOD;
          RCLCPP_WARN(rclcpp::get_logger("TeensySystem"),
            "Waiting for first Q... re-sending AUTO until Teensy starts streaming.");
        }
      }
    }
    return hardware_interface::return_type::OK;
  }

  // STARTUP HOLD
  {
    auto now = std::chrono::steady_clock::now();
    const bool holding =
      (g_hold_until[this] != std::chrono::steady_clock::time_point{} &&
       now < g_hold_until[this]);

    if (holding) {
      bool forced = false;

      if (g_startup_hold_strict[this]) {
        // strict: ALWAYS force command=state during hold
        for (size_t i = 0; i < commands_.size(); ++i) {
          if (std::isfinite(positions_[i])) {
            if (!std::isfinite(commands_[i]) || commands_[i] != positions_[i]) {
              commands_[i] = positions_[i];
              forced = true;
            }
          }
        }
      } else {
        // non-strict: only block large jumps
        for (size_t i = 0; i < commands_.size(); ++i) {
          if (std::isfinite(positions_[i]) && std::isfinite(commands_[i])) {
            if (std::fabs(commands_[i] - positions_[i]) > STARTUP_JUMP_RAD) {
              commands_[i] = positions_[i];
              forced = true;
            }
          } else if (std::isfinite(positions_[i])) {
            commands_[i] = positions_[i];
            forced = true;
          }
        }
      }

      if (forced) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("TeensySystem"),
          g_clock, 500,
          "Startup hold active: forcing command = state (prevents startup motion).");
      }
    }
  }

  auto & last_sent = g_last_hw_sent[this];
  if (last_sent.size() != info_.joints.size()) {
    last_sent.assign(info_.joints.size(), 0.0);
  }

  auto to_hw_cmd = [&](int idx) -> double {
    const size_t u = static_cast<size_t>(idx);

    if (g_io_is_ros[this]) {
      double v = commands_[u];
      if (!std::isfinite(v)) v = last_sent[u];
      else last_sent[u] = v;
      return v;
    }

    double ros = commands_[u] + g_bias[this][u];
    double v = ((ros + offset_[u]) * sign_[u]) * scale_[u];

    if (!std::isfinite(v)) {
      v = last_sent[u];
    } else {
      last_sent[u] = v;
    }
    return v;
  };

  const double tx_cont  = to_hw_cmd(idx_base_);
  const double tx_rev1  = to_hw_cmd(idx_arm1_);
  const double tx_rev2  = to_hw_cmd(idx_arm2_);
  const double tx_wrist = to_hw_cmd(idx_wrist_);

  char buf[128];
  int len = std::snprintf(
    buf, sizeof(buf), "J %.6f %.6f %.6f %.6f\r\n",
    tx_cont, tx_rev1, tx_rev2, tx_wrist);

  if (len <= 0 || len >= static_cast<int>(sizeof(buf))) {
    RCLCPP_ERROR(rclcpp::get_logger("TeensySystem"), "Failed to format command string");
    return hardware_interface::return_type::ERROR;
  }

  auto & pending = g_tx_pending[this];

  if (pending.size() > MAX_TX_PENDING_BYTES) {
    pending.clear();
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("TeensySystem"),
      g_clock, 2000,
      "TX backlog exceeded %zu bytes; dropping queued commands and keeping only latest.",
      MAX_TX_PENDING_BYTES);
  }

  pending.append(buf, static_cast<size_t>(len));
  (void)flush_pending(fd_, pending);

  int & tx_ctr = g_tx_log_counter[this];
  if ((tx_ctr++ % log_div_) == 0) {
    const std::string & last_rx = g_last_rx_line[this];

    RCLCPP_INFO(
      rclcpp::get_logger("TeensySystem"),
      "TX to Teensy: %.*s pending=%zuB  lastRX='%s'",
      len, buf,
      pending.size(),
      last_rx.c_str());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace teensy_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  teensy_hardware_interface::TeensySystem,
  hardware_interface::SystemInterface)

