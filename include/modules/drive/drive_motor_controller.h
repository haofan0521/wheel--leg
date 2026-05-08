#pragma once

#include <stdint.h>

#include <SimpleFOC.h>

#include "modules/drive/drive_pins.h"

namespace drive {

class DriveMotorController {
 public:
  using SensorProvider = Sensor* (*)();
  using MeasurementReader = float (*)();

  enum class TorqueMode : uint8_t {
    kVoltage = 0,
    kDcCurrent = 1,
    kFocCurrent = 2,
  };

  struct Config {
    const char* name;
    uint8_t pole_pairs;
    pins::ThreePhasePwmPins pwm_pins;
    pins::FeedbackPins feedback_pins;
    SensorProvider sensor_provider;
    MeasurementReader read_angle;
    MeasurementReader read_velocity;
    uint8_t pwm_channel_a;
    uint8_t pwm_channel_b;
    uint8_t pwm_channel_c;
    float velocity_direction;
  };

  struct Status {
    bool initialized;
    bool foc_ready;
    bool enabled;
    bool emergency_stopped;
    bool open_loop;
    TorqueMode torque_mode;
    bool current_sense_ready;
    bool simplefoc_current_sense_ready;
    bool over_current;
    float target_velocity;
    float measured_velocity;
    float shaft_angle;
    float voltage_limit;
    float velocity_p;
    float velocity_i;
    float velocity_d;
    float velocity_lpf_tf;
    float velocity_error;
    float velocity_p_output;
    float velocity_i_output;
    float velocity_pid_output;
    float current_limit;
    float current_q;
    float current_d;
    float current_sp;
    float voltage_q;
    float voltage_d;
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float phase_voltage_b;
    float phase_voltage_c;
    float phase_offset_voltage_b;
    float phase_offset_voltage_c;
    float phase_voltage_delta_b;
    float phase_voltage_delta_c;
  };

  explicit DriveMotorController(const Config& config);

  void init();
  void update();
  void processSerial();
  void setOpenLoop(bool open_loop);
  void setEnabled(bool enabled);
  void setTargetVelocity(float velocity);
  void emergencyStop();
  void setVoltageLimit(float limit);
  void setVelocityPid(float p, float i, float d, float lpf_tf);
  void setCurrentLimit(float limit);
  void setTorqueMode(TorqueMode mode);
  float getAngle();
  Status status();

 private:
  float clampVoltageLimit(float limit) const;
  void configureOpenLoopPwm();
  void writeOpenLoopDuty(float phase_a, float phase_b, float phase_c);
  void centerOpenLoopPwm();
  void updateOpenLoopPwm();
  void applyVelocityTuning();
  void resetVelocityTelemetry();
  void updateVelocityTelemetry(float measured_velocity);
  void configureMotorForCurrentMode();
  TorqueControlType simpleFocTorqueMode() const;
  const char* torqueModeName() const;
  bool initializeClosedLoopFoc();
  void disableMotorOutput();
  void configureCurrentSensePins();
  void calibrateCurrentOffsets();
  void updateCurrentSample();
  float readAdcVoltage(uint8_t pin) const;

  Config config_;
  BLDCMotor motor_;
  BLDCDriver3PWM driver_;
  InlineCurrentSense current_sense_;
  float target_velocity_;
  float open_loop_voltage_limit_;
  float velocity_p_;
  float velocity_i_;
  float velocity_d_;
  float velocity_lpf_tf_;
  float open_loop_electrical_angle_;
  uint32_t last_open_loop_update_us_;
  bool initialized_;
  bool foc_ready_;
  bool enabled_;
  bool emergency_stopped_;
  bool open_loop_;
  bool sensor_available_;
  bool foc_attempted_;
  TorqueMode torque_mode_;
  bool current_sense_ready_;
  bool simplefoc_current_sense_ready_;
  bool over_current_;
  float current_limit_;
  float velocity_error_;
  float velocity_p_output_;
  float velocity_i_output_;
  float velocity_pid_output_;
  float velocity_error_prev_;
  uint32_t velocity_telemetry_update_us_;
  float phase_current_a_;
  float phase_current_b_;
  float phase_current_c_;
  float phase_voltage_b_;
  float phase_voltage_c_;
  float phase_offset_voltage_b_;
  float phase_offset_voltage_c_;
  float phase_voltage_delta_b_;
  float phase_voltage_delta_c_;
};

}  // namespace drive
