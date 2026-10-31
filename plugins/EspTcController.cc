// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "EspTcController.hh"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <cmath>
#include <algorithm>

using namespace gz;
using namespace sim;
using namespace systems;

static constexpr double kDeg2Rad = M_PI / 180.0;
static constexpr double kRad2Deg = 180.0 / M_PI;

void EspTcController::initPids()
{
  double dt = 1.0 / controlFrequency;
  pidRoll.init(5.3, 2.5, 0.0, dt);  pidRoll.iLimit  = 20.0;
  pidPitch.init(5.3, 2.5, 0.0, dt); pidPitch.iLimit = 20.0;
  pidYaw.init(6.0, 1.0, 0.35, dt);  pidYaw.iLimit   = 360.0;
}

void EspTcController::resetAll()
{
  pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
  nu_phi = nu_theta = nu_psi = 0.0;
  tau_phi_f = tau_theta_f = tau_psi_f = 0.0;
  phip_f_ = thetap_f_ = psip_f_ = 0.0;
}

void EspTcController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  model = Model(_entity);
  if (!model.Valid(_ecm)) { gzerr << "EspTcController: invalid model\n"; return; }

  if (_sdf->HasElement("comLinkName"))      comLinkName      = _sdf->Get<std::string>("comLinkName");
  if (_sdf->HasElement("controlFrequency")) controlFrequency = _sdf->Get<double>("controlFrequency");
  if (_sdf->HasElement("thrustBase"))       thrustBase       = _sdf->Get<double>("thrustBase");
  if (_sdf->HasElement("lpf_cutoff_hz")) {
    double wc = 2.0 * M_PI * _sdf->Get<double>("lpf_cutoff_hz") / controlFrequency;
    lpf_alpha = wc / (wc + 1.0);
  }
  if (_sdf->HasElement("rate_lpf_cutoff_hz")) {
    double wc = 2.0 * M_PI * _sdf->Get<double>("rate_lpf_cutoff_hz") / controlFrequency;
    rate_lpf_alpha_ = wc / (wc + 1.0);
  }

  initPids();

  if (_sdf->HasElement("k0_phi")) k0_phi = _sdf->Get<double>("k0_phi");
  if (_sdf->HasElement("k1_phi")) k1_phi = _sdf->Get<double>("k1_phi");
  if (_sdf->HasElement("k2_phi")) k2_phi = _sdf->Get<double>("k2_phi");
  if (_sdf->HasElement("k0_theta")) k0_theta = _sdf->Get<double>("k0_theta");
  if (_sdf->HasElement("k1_theta")) k1_theta = _sdf->Get<double>("k1_theta");
  if (_sdf->HasElement("k2_theta")) k2_theta = _sdf->Get<double>("k2_theta");
  if (_sdf->HasElement("k0_psi")) k0_psi = _sdf->Get<double>("k0_psi");
  if (_sdf->HasElement("k1_psi")) k1_psi = _sdf->Get<double>("k1_psi");
  if (_sdf->HasElement("k2_psi")) k2_psi = _sdf->Get<double>("k2_psi");
  if (_sdf->HasElement("nu_max")) nu_max = _sdf->Get<double>("nu_max");
  if (_sdf->HasElement("k3_phi")) k3_phi = _sdf->Get<double>("k3_phi");
  if (_sdf->HasElement("k3_theta")) k3_theta = _sdf->Get<double>("k3_theta");
  if (_sdf->HasElement("k3_psi")) k3_psi = _sdf->Get<double>("k3_psi");
  if (_sdf->HasElement("ftsmo_D_att")) {
    const double bound = _sdf->Get<double>("ftsmo_D_att");
    obsPhi.setBound(bound); obsTheta.setBound(bound); obsPsi.setBound(bound);
  }
  std::string ns = _sdf->HasElement("robotNamespace")
                   ? _sdf->Get<std::string>("robotNamespace")
                   : model.Name(_ecm);

  node.Subscribe("/" + ns + "/cmd_attitude", &EspTcController::OnCmdAttitude, this);
  node.Subscribe("/" + ns + "/pid_gains",    &EspTcController::OnPidGains,    this);
  node.Subscribe("/" + ns + "/tc_gains",     &EspTcController::OnTcGains,     this);

  controlCmdPub = node.Advertise<gz::msgs::Twist>("/" + ns + "/control_cmd");
  statePub = node.Advertise<gz::msgs::Float_V>("/" + ns + "/controller_state");

  std::chrono::duration<double> period{1.0 / controlFrequency};
  controlPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  gzmsg << "EspTcController on " << ns << " @ " << controlFrequency << " Hz\n";
}

void EspTcController::OnCmdAttitude(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lk(cmdMutex);
  cmd.thrust   = _msg.linear().z();
  cmd.rollDes  = _msg.angular().x();
  cmd.pitchDes = _msg.angular().y();
  cmd.yawDes   = _msg.angular().z();
}

void EspTcController::OnPidGains(const gz::msgs::Float_V &_msg)
{
  if (_msg.data_size() < 9) {
    gzwarn << "EspTcController: pid_gains needs 9 floats, got " << _msg.data_size() << "\n";
    return;
  }
  std::lock_guard<std::mutex> lk(gainsMutex);
  pidRoll.kp  = _msg.data(0); pidRoll.ki  = _msg.data(1); pidRoll.kd  = _msg.data(2);
  pidPitch.kp = _msg.data(3); pidPitch.ki = _msg.data(4); pidPitch.kd = _msg.data(5);
  pidYaw.kp   = _msg.data(6); pidYaw.ki   = _msg.data(7); pidYaw.kd   = _msg.data(8);
  pidGainsUpdated = true;
}

void EspTcController::OnTcGains(const gz::msgs::Float_V &_msg)
{
  if (_msg.data_size() < 13) {
    gzwarn << "EspTcController: tc_gains needs 13 floats, got " << _msg.data_size() << "\n";
    return;
  }
  std::lock_guard<std::mutex> lk(gainsMutex);
  k0_phi=_msg.data(0);  k1_phi=_msg.data(1);  k2_phi=_msg.data(2);  k3_phi=_msg.data(3);
  k0_theta=_msg.data(4);k1_theta=_msg.data(5);k2_theta=_msg.data(6);k3_theta=_msg.data(7);
  k0_psi=_msg.data(8);  k1_psi=_msg.data(9);  k2_psi=_msg.data(10); k3_psi=_msg.data(11);
  nu_max=_msg.data(12);
  tcGainsUpdated = true;
}

void EspTcController::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (_info.paused) return;

  if (!initialized) {
    bodyEntity = model.LinkByName(_ecm, comLinkName);
    if (bodyEntity == kNullEntity) {
      gzerr << "EspTcController: link [" << comLinkName << "] not found\n"; return;
    }
    Link(bodyEntity).EnableVelocityChecks(_ecm, true);
    initialized = true;
  }

  {
    std::lock_guard<std::mutex> lk(gainsMutex);
    if (pidGainsUpdated || tcGainsUpdated) {
      resetAll();
      pidGainsUpdated = tcGainsUpdated = false;
    }
  }

  if (_info.simTime < lastControlTime) {
    lastControlTime = std::chrono::steady_clock::duration{0};
    resetAll();
    std::lock_guard<std::mutex> lk(cmdMutex);
    cmd = CmdAttitude{thrustBase, 0, 0, 0};
  }

  if (_info.simTime - lastControlTime < controlPeriod) return;
  lastControlTime = _info.simTime;

  Link bodyLink(bodyEntity);
  auto poseOpt   = bodyLink.WorldPose(_ecm);
  auto angVelOpt = bodyLink.WorldAngularVelocity(_ecm);
  if (!poseOpt || !angVelOpt) return;

  const auto &pose        = poseOpt.value();
  const auto &angVelWorld = angVelOpt.value();

  double rollActual  = pose.Rot().Roll()  * kRad2Deg;
  double pitchActual = pose.Rot().Pitch() * kRad2Deg;
  double yawActual   = pose.Rot().Yaw()   * kRad2Deg;

  math::Vector3d angVelBody = pose.Rot().Inverse().RotateVector(angVelWorld);
  double rollRateActual  = angVelBody.X() * kRad2Deg;
  double pitchRateActual = angVelBody.Y() * kRad2Deg;
  double yawRateActual   = angVelBody.Z() * kRad2Deg;

  CmdAttitude sp;
  { std::lock_guard<std::mutex> lk(cmdMutex); sp = cmd; }

  // === Outer attitude PID ===
  pidRoll.desired  = sp.rollDes;
  double rollRateDesired  = pidRoll.update(rollActual, true);

  pidPitch.desired = sp.pitchDes;
  double pitchRateDesired = pidPitch.update(pitchActual, true);

  pidYaw.error = wrapYaw180(sp.yawDes - yawActual);
  double yawRateDesired   = pidYaw.update(yawActual, false);

  // === TC inner loop (all in radians) ===
  const double dt = 1.0 / controlFrequency;

  // Without a thrust command the drone is on the ground, where the attitude
  // error cannot be corrected. Integrating it would saturate nu before take off.
  if (sp.thrust < 1.0) {
    nu_phi = nu_theta = nu_psi = 0.0;
    tau_phi_f = tau_theta_f = tau_psi_f = 0.0;
  }

  double phi   = rollActual  * kDeg2Rad,  theta = pitchActual * kDeg2Rad,  psi = yawActual  * kDeg2Rad;
  double phid  = sp.rollDes  * kDeg2Rad,  thetad= sp.pitchDes * kDeg2Rad,  psid= sp.yawDes  * kDeg2Rad;
  double phidp   = rollRateDesired  * kDeg2Rad;
  double thetadp = pitchRateDesired * kDeg2Rad;
  double psidp   = yawRateDesired   * kDeg2Rad;
  double phip    = rollRateActual   * kDeg2Rad;
  double thetap  = pitchRateActual  * kDeg2Rad;
  double psip    = yawRateActual    * kDeg2Rad;
  phip_f_   = (1.0 - rate_lpf_alpha_) * phip_f_   + rate_lpf_alpha_ * phip;
  thetap_f_ = (1.0 - rate_lpf_alpha_) * thetap_f_ + rate_lpf_alpha_ * thetap;
  psip_f_   = (1.0 - rate_lpf_alpha_) * psip_f_   + rate_lpf_alpha_ * psip;
  phip = phip_f_; thetap = thetap_f_; psip = psip_f_;

  double ephi   = phid  - phi,   etheta = thetad - theta,  epsi  = psid  - psi;
  double ephip  = phidp - phip,  ethetap= thetadp- thetap, epsip = psidp - psip;

  // Integrators: ν += (k2·sign(e) + k3·sign(ė)) · dt
  nu_phi   = std::clamp(nu_phi   + (k2_phi   * sgn(ephi)   + k3_phi   * sgn(ephip))   * dt, -nu_max, nu_max);
  nu_theta = std::clamp(nu_theta + (k2_theta * sgn(etheta) + k3_theta * sgn(ethetap)) * dt, -nu_max, nu_max);
  nu_psi   = std::clamp(nu_psi   + (k2_psi   * sgn(epsi)   + k3_psi   * sgn(epsip))   * dt, -nu_max, nu_max);

  // τ = ν + k0·|e|^(1/3)·sign(e) + k1·|ė|^(1/2)·sign(ė)
  // Epsilon guard on |e|^(1/3): exponent < 1, derivative diverges at 0
  double tau_phi   = nu_phi   + k0_phi   * std::pow(std::abs(ephi)   + 1e-6, 1.0/3.0) * sgn(ephi)
                               + k1_phi   * std::pow(std::abs(ephip)  + 1e-6, 0.5)     * sgn(ephip);
  double tau_theta = nu_theta + k0_theta * std::pow(std::abs(etheta) + 1e-6, 1.0/3.0) * sgn(etheta)
                               + k1_theta * std::pow(std::abs(ethetap)+ 1e-6, 0.5)     * sgn(ethetap);
  double tau_psi   = nu_psi   + k0_psi   * std::pow(std::abs(epsi)   + 1e-6, 1.0/3.0) * sgn(epsi)
                               + k1_psi   * std::pow(std::abs(epsip)  + 1e-6, 0.5)     * sgn(epsip);

  if (obsPhi.enabled) {
    if (sp.thrust < 1.0) {
      obsPhi.reset(phi); obsTheta.reset(theta); obsPsi.reset(psi);
    } else {
      tau_phi   -= obsPhi.update(phi, kAccelPerUnitRoll * tau_phi_f, dt) / kAccelPerUnitRoll;
      tau_theta -= obsTheta.update(theta, kAccelPerUnitPitch * tau_theta_f, dt) / kAccelPerUnitPitch;
      tau_psi   -= obsPsi.update(psi, kAccelPerUnitYaw * tau_psi_f, dt, true) / kAccelPerUnitYaw;
    }
  }

  tau_phi   = std::clamp(tau_phi,   -32000.0, 32000.0);
  tau_theta = std::clamp(tau_theta, -32000.0, 32000.0);
  tau_psi   = std::clamp(tau_psi,   -32000.0, 32000.0);

  tau_phi_f   = (1.0 - lpf_alpha) * tau_phi_f   + lpf_alpha * tau_phi;
  tau_theta_f = (1.0 - lpf_alpha) * tau_theta_f + lpf_alpha * tau_theta;
  tau_psi_f   = (1.0 - lpf_alpha) * tau_psi_f   + lpf_alpha * tau_psi;

  // Publish filtered actuator signals to EspPowerDistribution for motor mixing
  gz::msgs::Twist controlCmdMsg;
  controlCmdMsg.mutable_linear()->set_z(sp.thrust);
  controlCmdMsg.mutable_angular()->set_x(tau_phi_f);
  controlCmdMsg.mutable_angular()->set_y(tau_theta_f);
  controlCmdMsg.mutable_angular()->set_z(tau_psi_f);
  controlCmdPub.Publish(controlCmdMsg);

  double t = std::chrono::duration<double>(_info.simTime).count();
  gz::msgs::Float_V stateMsg;
  for (double v : {t, rollActual, pitchActual, yawActual,
                   rollRateActual, pitchRateActual, yawRateActual})
    stateMsg.add_data(static_cast<float>(v));
  statePub.Publish(stateMsg);
}

GZ_ADD_PLUGIN(EspTcController, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(EspTcController, "gz::sim::systems::EspTcController")
