// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "EspControllerCommon.hh"
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
 * Non-Singular Terminal Sliding Mode Controller (NTSMC) attitude controller.
 *
 * Outer loop : attitude PID (deg → deg/s) — identical to EspAttitudeController.
 * Inner loop : NTSMC in rad/rad·s units.
 *
 * Sliding surface  S_i = e_i + k0_i · |ė_i|^(3/2) · sign(ė_i)
 * Integrator       ν_i += k2_i · sign(S_i) · dt   (clamped ±ν_max)
 * Control law      u_i = ν_i + k1_i · |S_i|^(1/3) · sign(S_i)   (k1,k2,ν in PWM units)
 *
 * Topics
 *   sub  /{ns}/cmd_attitude    (gz.msgs.Twist)
 *   sub  /{ns}/pid_gains       (gz.msgs.Float_V)  — 9 floats: roll kp,ki,kd  pitch kp,ki,kd  yaw kp,ki,kd
 *   sub  /{ns}/ntsmc_gains     (gz.msgs.Float_V)  — 10 floats: k0,k1,k2 per phi/theta/psi + nu_max
 *   pub  /{ns}/motor_command   (gz.msgs.Actuators)
 *   pub  /{ns}/controller_state (gz.msgs.Float_V)
 */
class EspNtsmcController : public System,
                           public ISystemConfigure,
                           public ISystemPreUpdate
{
public:
  EspNtsmcController()  = default;
  ~EspNtsmcController() override = default;

  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &, EventManager &) override;
  void PreUpdate(const UpdateInfo &, EntityComponentManager &) override;

private:
  void OnCmdAttitude(const gz::msgs::Twist &_msg);
  void OnPidGains(const gz::msgs::Float_V &_msg);
  void OnNtsmcGains(const gz::msgs::Float_V &_msg);
  void initPids();
  void resetAll();

  Model  model{kNullEntity};
  Entity bodyEntity{kNullEntity};
  std::string comLinkName{"body"};

  AttPidObject pidRoll, pidPitch, pidYaw;

  // NTSMC gains
  double k0_phi{7.699619},  k1_phi{692.2917},    k2_phi{660.3927};
  double k0_theta{7.699619},k1_theta{692.2917},   k2_theta{660.3927};
  double k0_psi{9.287063},  k1_psi{7518.5885},   k2_psi{1779.8053};
  double nu_max{6312.3157};

  double nu_phi{0}, nu_theta{0}, nu_psi{0};

  // LPF state on output torque (default 40 Hz @ 500 Hz)
  double tau_phi_f{0.0}, tau_theta_f{0.0}, tau_psi_f{0.0};
  double lpf_alpha{0.3345};

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
  bool ntsmcGainsUpdated{false};
};

}}} // namespace gz::sim::systems
