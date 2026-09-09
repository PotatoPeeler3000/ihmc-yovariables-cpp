#pragma once

#include <algorithm>
#include <cmath>

#include "ihmc/yovariables/providers/double_provider.h"

namespace ihmc::yovariables::filters
{
/**
 * Small internal support utilities, not ported from any single Java file: substitutes for the
 * trivial pieces of us.ihmc.commons.{AngleTools,MathTools}, us.ihmc.euclid.tools.EuclidCoreTools,
 * and us.ihmc.commons.DeadbandTools that the filters package calls, none of which are part of
 * ihmc-yovariables itself and so are not otherwise being ported.
 */

inline constexpr double kTwoPi = 2.0 * M_PI;

inline double clamp(double value, double min, double max)
{
   return std::min(std::max(value, min), max);
}

/** Clamps value to [-max, max]. */
inline double clampSymmetric(double value, double max)
{
   return clamp(value, -max, max);
}

/** (1 - alpha) * a + alpha * b. */
inline double interpolate(double a, double b, double alpha)
{
   return (1.0 - alpha) * a + alpha * b;
}

inline bool epsilonEquals(double a, double b, double epsilon)
{
   return std::abs(a - b) <= epsilon;
}

/** a - b, wrapped to (-pi, pi]. */
inline double angleDifferenceMinusPiToPi(double a, double b)
{
   double difference = std::fmod(a - b, kTwoPi);
   if (difference <= -M_PI)
      difference += kTwoPi;
   else if (difference > M_PI)
      difference -= kTwoPi;
   return difference;
}

/** Standard deadband: 0 within [-deadband, deadband], else value shifted toward 0 by deadband. */
inline double applyDeadband(double deadband, double value)
{
   if (value > deadband)
      return value - deadband;
   if (value < -deadband)
      return value + deadband;
   return 0.0;
}

/** Adapts a fixed double to the DoubleProvider interface - C++ equivalent of Java's `() -> dt`. */
class ConstantDoubleProvider : public providers::DoubleProvider
{
public:
   explicit ConstantDoubleProvider(double value = 0.0) : value_(value)
   {
   }

   double getValue() const override
   {
      return value_;
   }

   void setValue(double value)
   {
      value_ = value;
   }

private:
   double value_;
};
}
