#pragma once
#include <algorithm>
#include <cmath>

inline double clamp(double v, double lo, double hi){
  return std::max(lo, std::min(hi, v));
}

inline double apply_deadzone(double v, double deadzone){
  if (deadzone <= 0.0) return v;
  return (std::fabs(v) < deadzone) ? 0.0 : v;
}
