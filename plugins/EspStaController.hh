// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "EspControllerCommon.hh"
#include "EspFtSmo.hh"
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <mutex>
#include <string>

namespace gz { namespace sim { namespace systems {

/*
 * Super-Twisting Algorithm (STA) attitude controller.
 *
 * Outer loop : attitude PID (deg → deg/s) — identical to EspAttitudeController.
 * Inner loop : STA in rad/rad·s units.
 *
 * Sliding surface  S_i = ė_i + k0_i · e_i
 * Integrator       ν_i += k2_i · sign(S_i) · dt    (clamped ±ν_max)
 * Control law      τ_i = k1_i · √|S_i| · sign(S_i) + ν_i   (k1,k2,ν in PWM units)
 *
 * Topics
 *   sub  /{ns}/cmd_attitude   (gz.msgs.Twist)    — thrust + desired roll/pitch/yaw [deg]
 *   sub  /{ns}/pid_gains      (gz.msgs.Float_V)  — 9 floats: roll kp,ki,kd  pitch kp,ki,kd  yaw kp,ki,kd
 *   sub  /{ns}/sta_gains      (gz.msgs.Float_V)  — 10 floats: k0,k1,k2 per phi/theta/psi + nu_max
 *   pub  /{ns}/motor_command  (gz.msgs.Actuators)
 *   pub  /{ns}/controller_state (gz.msgs.Float_V) — [t, roll, pitch, yaw, rollRate, pitchRate, yawRate]
 */
class EspStaController : public System,
                         public ISystemConfigure,
                         public ISystemPreUpdate
{
public:
  EspStaController()  = default;
  ~EspStaController() override = default;

  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &, EventManager &) override;
  void PreUpdate(const UpdateInfo &, EntityComponentManager &) override;

private:
  void OnCmdAttitude(const gz::msgs::Twist &_msg);
  void OnPidGains(const gz::msgs::Float_V &_msg);
  void OnStaGains(const gz::msgs::Float_V &_msg);
  void initPids();
  void resetAll();

  Model  model{kNullEntity};
  Entity bodyEntity{kNullEntity};
  std::string comLinkName{"body"};

  // Outer attitude PID (angle → rate setpoint)
  AttPidObject pidRoll, pidPitch, pidYaw;

  // STA gains (phi = roll, theta = pitch, psi = yaw)
  double k0_phi{0.023029},  k1_phi{552.3170},    k2_phi{572.9733};
  double k0_theta{0.023029},k1_theta{552.3170},   k2_theta{572.9733};
  double k0_psi{0.001523},  k1_psi{12545.9658},  k2_psi{8663.0126};
  double nu_max{21289.5471};

  // STA integrator state
  double nu_phi{0}, nu_theta{0}, nu_psi{0};

  // LPF state on output torque (default 40 Hz @ 500 Hz)
  double tau_phi_f{0.0}, tau_theta_f{0.0}, tau_psi_f{0.0};
  double lpf_alpha{0.3345};
  // Angular acceleration per unit of control output, from the motor and frame model.
  static constexpr double kAccelPerUnitRoll{0.0615};
  static constexpr double kAccelPerUnitPitch{0.0675};
  static constexpr double kAccelPerUnitYaw{0.0098};
  espdrone::EspFtSmo obsPhi, obsTheta, obsPsi;

  // LPF state on angular rate measurements (default 50 Hz @ 500 Hz)
  double phip_f_{0.0}, thetap_f_{0.0}, psip_f_{0.0};
  double rate_lpf_alpha_{0.386};

  double controlFrequency{500.0};
  double thrustBase{42000.0};
  std::chrono::steady_clock::duration controlPeriod;
  std::chrono::steady_clock::duration lastControlTime{0};

  CmdAttitude cmd;
  std::mutex  cmdMutex;

  gz::transport::Node node;
  gz::transport::Node::Publisher controlCmdPub;
  gz::transport::Node::Publisher statePub;

  bool initialized{false};
  std::mutex gainsMutex;
  bool pidGainsUpdated{false};
  bool staGainsUpdated{false};
};

}}} // namespace gz::sim::systems
