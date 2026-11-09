#ifndef STS_HARDWARE_INTERFACE_STS_CONVERSIONS_HPP_
#define STS_HARDWARE_INTERFACE_STS_CONVERSIONS_HPP_

/**
 * @file sts_conversions.hpp
 * @brief Pure conversion free functions for STS servo motor unit translation.
 *
 * All functions are stateless and depend only on their arguments and the
 * compile-time protocol constants below. Extracted here so they can be
 * independently unit-tested without instantiating STSHardwareInterface.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace sts_hardware_interface
{
namespace conversions
{

// `sign` (default -1) is the joint direction relative to the raw encoder: -1 inverts it
// (REP-103, motor clockwise-positive), +1 keeps it (rising raw step = positive angle).

// ===== STS protocol constants (same for all STS motors) =====
constexpr double STEPS_PER_REVOLUTION = 4096.0;
constexpr double STEPS_TO_RAD = (2.0 * M_PI) / STEPS_PER_REVOLUTION;
constexpr double RAD_TO_STEPS = STEPS_PER_REVOLUTION / (2.0 * M_PI);
constexpr int    STS_MAX_POSITION     = 4095;   // 12-bit encoder
constexpr int    STS_DEFAULT_CENTER   = 4095;   // Default: 0 rad at step 4095, range [0, 2π)
constexpr int    STS_MAX_PWM          = 1000;   // ±100% duty cycle
constexpr int    STS_MAX_ACCELERATION = 254;    // protocol constant
constexpr int    STS_MIDPOINT_RAW_POSITION = 2048;  // Encoder midpoint; one-key calibration (CalibrationOfs) resets current position to this raw step

/**
 * @brief Convert motor steps [0, 4095] to radians with configurable center.
 *
 * STS motors increment clockwise; ROS 2 uses counter-clockwise positive (REP-103).
 * `center` is the raw encoder step that maps to 0 rad (default: STS_DEFAULT_CENTER = 4095).
 */
inline double raw_position_to_radians(int raw_position, int center = STS_DEFAULT_CENTER, int sign = -1)
{
  return static_cast<double>(sign * (raw_position - center)) * STEPS_TO_RAD;
}

/**
 * @brief Convert radians to motor steps [0, 4095] with configurable center.
 *
 * `center` is the raw encoder step that maps to 0 rad (default: STS_DEFAULT_CENTER = 4095).
 * Values outside the physical encoder range are clamped (not wrapped) to [0, 4095].
 * Clamping is intentional: the old fmod-based wrapping silently mapped negative
 * angles to the wrong end of the encoder, causing full-revolution traversals.
 */
inline int radians_to_raw_position(double position_rad, int center = STS_DEFAULT_CENTER, int sign = -1)
{
  double steps = std::round(static_cast<double>(center) + sign * position_rad * RAD_TO_STEPS);
  return static_cast<int>(std::clamp(steps, 0.0, static_cast<double>(STS_MAX_POSITION)));
}

/**
 * @brief Convert raw velocity (steps/s) to rad/s with sign inversion.
 *
 * Motor positive direction is clockwise; ROS 2 uses counter-clockwise positive.
 */
inline double raw_velocity_to_rad_s(int raw_velocity, int sign = -1)
{
  return static_cast<double>(sign * raw_velocity) * STEPS_TO_RAD;
}

/**
 * @brief Convert rad/s to raw velocity (steps/s), clamped to ±max_velocity_steps.
 *
 * Applies the same sign inversion as raw_velocity_to_rad_s.
 * Use this for MODE_VELOCITY (WriteSpe / SyncWriteSpe) where sign encodes direction.
 */
inline int rad_s_to_raw_velocity(double velocity_rad_s, int max_velocity_steps, int sign = -1)
{
  double raw = sign * velocity_rad_s * RAD_TO_STEPS;
  return static_cast<int>(std::clamp(
    raw,
    static_cast<double>(-max_velocity_steps),
    static_cast<double>(max_velocity_steps)));
}

/**
 * @brief Convert a speed magnitude (rad/s) to raw steps/s for position mode.
 *
 * Unlike rad_s_to_raw_velocity, this does NOT apply sign inversion.
 * WritePosEx / SyncWritePosEx take speed as an unsigned magnitude — direction
 * is implied by the position target.  Returns 0 when speed_mag_rad_s == 0,
 * which the STS protocol interprets as "no speed limit / use hardware max".
 *
 * Use this for MODE_SERVO (WritePosEx / SyncWritePosEx) only.
 */
inline int rad_s_to_raw_speed(double speed_mag_rad_s, int max_velocity_steps)
{
  return static_cast<int>(std::clamp(
    speed_mag_rad_s * RAD_TO_STEPS,
    0.0,
    static_cast<double>(max_velocity_steps)));
}

/**
 * @brief Convert normalized effort [-1.0, +1.0] to motor PWM [-1000, +1000].
 *
 * Applies the same sign inversion as raw_velocity_to_rad_s.
 */
inline int effort_to_raw_pwm(double effort, int sign = -1)
{
  return static_cast<int>(std::clamp(sign * effort, -1.0, 1.0) * STS_MAX_PWM);
}

/**
 * @brief Clamp an acceleration command value to the valid protocol range [0, 254].
 */
inline int clamp_acceleration(double accel_cmd)
{
  return static_cast<int>(
    std::clamp(accel_cmd, 0.0, static_cast<double>(STS_MAX_ACCELERATION)));
}

/**
 * @brief Clamp effort to [-max_effort, max_effort] safety limit.
 *
 * Acts as a limiter without scaling: if max_effort=0.5 and command=0.5,
 * the output is 0.5 (not rescaled to 1.0).
 */
inline double normalize_effort(double effort, double max_effort, bool has_limit)
{
  if (!has_limit) return effort;
  return std::clamp(effort, -max_effort, max_effort);
}

/**
 * @brief Clamp value to [min_val, max_val] when limit is enabled; pass through otherwise.
 */
template<typename T>
inline T apply_limit(T value, T min_val, T max_val, bool has_limit)
{
  return has_limit ? std::clamp(value, min_val, max_val) : value;
}

/**
 * @brief Decode the STS/SCS servo status byte (SCSerial::Error, as set by getErr()) into a
 * human-readable fault summary.
 *
 * Per the Feetech SMS/STS servo memory table, only bits 0, 2, and 5 are defined; all other
 * bits are reserved and always 0 on STS-series servos.
 *   Bit 0: Voltage error   (supply voltage outside the servo's configured min/max range)
 *   Bit 2: Overheat error  (internal temperature exceeds the configured max temperature)
 *   Bit 5: Overload error  (load exceeded overload_torque for longer than the overload time)
 *
 * @return "OK" if error_byte == 0, otherwise a comma-separated list of active faults.
 */
inline std::string decode_servo_error(uint8_t error_byte)
{
  if (error_byte == 0) return "OK";

  std::string result;
  auto append_fault = [&](const char * name) {
    if (!result.empty()) result += ", ";
    result += name;
  };
  if (error_byte & (1 << 0)) append_fault("voltage error");
  if (error_byte & (1 << 2)) append_fault("overheat error");
  if (error_byte & (1 << 5)) append_fault("overload error");

  // Any remaining set bits are reserved/undocumented on STS-series servos; surface them
  // rather than silently dropping the information.
  constexpr uint8_t known_mask = (1 << 0) | (1 << 2) | (1 << 5);
  if (uint8_t unknown_bits = error_byte & ~known_mask; unknown_bits != 0) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "unknown bits 0x%02x", unknown_bits);
    append_fault(buf);
  }

  return result;
}

}  // namespace conversions
}  // namespace sts_hardware_interface

#endif  // STS_HARDWARE_INTERFACE_STS_CONVERSIONS_HPP_
