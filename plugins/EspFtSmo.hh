// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef GZ_SIM_SYSTEMS_ESPFTSMO_HH_
#define GZ_SIM_SYSTEMS_ESPFTSMO_HH_

#include <algorithm>
#include <cmath>

namespace espdrone
{

/// One axis of the finite-time sliding mode observer of the COMRob 2023 paper:
///   x1' = x2 + k1 |e|^(2/3) sign(e)
///   x2' = model + x3 + k2 |e|^(1/3) sign(e)
///   x3' = k3 sign(e)
/// with e = y - x1. The state x3 estimates the disturbance. The gains follow
/// the rule of the paper from the disturbance bound D.
class EspFtSmo
{
public:
  void setBound(double _d)
  {
    enabled = _d > 0.0;
    k1 = 2.0 * std::cbrt(_d);
    k2 = 1.5 * std::sqrt(_d);
    k3 = 1.1 * _d;
    limit = 4.0 * _d;
  }

  void reset(double _y)
  {
    x1 = _y; x2 = 0.0; x3 = 0.0;
  }

  /// _model is the acceleration the model expects from the control input.
  /// _wrap makes the error of an angle stay within +/- pi.
  double update(double _y, double _model, double _dt, bool _wrap = false)
  {
    double e = _y - x1;
    if (_wrap)
      e = std::atan2(std::sin(e), std::cos(e));
    const double s = (e > 0.0) - (e < 0.0);
    const double a = std::abs(e);
    x1 += (x2 + k1 * std::pow(a, 2.0 / 3.0) * s) * _dt;
    x2 += (_model + x3 + k2 * std::cbrt(a) * s) * _dt;
    x3 = std::clamp(x3 + k3 * s * _dt, -limit, limit);
    return x3;
  }

  bool enabled{false};
  double x1{0}, x2{0}, x3{0};

private:
  double k1{0}, k2{0}, k3{0}, limit{0};
};

}  // namespace espdrone

#endif
