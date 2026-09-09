#include "ihmc/yovariables/filters/first_order_filtered_yo_double.h"

#include <cmath>
#include <stdexcept>

namespace ihmc::yovariables::filters
{
namespace
{
std::string cutoffFrequencyName(const std::string& name, FirstOrderFilteredYoDouble::FirstOrderFilterType type)
{
   switch (type)
   {
      case FirstOrderFilteredYoDouble::FirstOrderFilterType::LOW_PASS:
         return name + "_LowPassCutoff_Hz";
      case FirstOrderFilteredYoDouble::FirstOrderFilterType::HIGH_PASS:
         return name + "_HighPassCutoff_Hz";
   }
   throw std::invalid_argument("Must specify filter type as low or high pass.");
}
} // namespace

FirstOrderFilteredYoDouble::FirstOrderFilteredYoDouble(const std::string& name, const std::string& description, double cutoffFrequencyHz,
                                                         providers::DoubleProvider* yoTime, FirstOrderFilterType highOrLowPass,
                                                         registry::YoRegistry* registry)
   : YoDouble(name, description, registry), cutoffFrequencyHz_(cutoffFrequencyName(name, highOrLowPass), registry), yoTime_(yoTime),
     filterType_(highOrLowPass)
{
   cutoffFrequencyHz_.set(cutoffFrequencyHz);
}

FirstOrderFilteredYoDouble::FirstOrderFilteredYoDouble(const std::string& name, const std::string& description, double cutoffFrequencyHz, double dt,
                                                         FirstOrderFilterType highOrLowPass, registry::YoRegistry* registry)
   : FirstOrderFilteredYoDouble(name, description, cutoffFrequencyHz, nullptr, highOrLowPass, registry)
{
   dt_ = dt;
}

double FirstOrderFilteredYoDouble::computeLowPassUpdate(double filterInput, double dt)
{
   double alpha = computeAlpha(dt, cutoffFrequencyHz_.getDoubleValue());
   return alpha * getDoubleValue() + (1.0 - alpha) * filterInput;
}

double FirstOrderFilteredYoDouble::computeHighPassUpdate(double filterInput, double dt)
{
   double alpha = computeAlpha(dt, cutoffFrequencyHz_.getDoubleValue());
   return alpha * (getDoubleValue() + filterInput - filterInputOld_);
}

double FirstOrderFilteredYoDouble::computeAlpha(double dt, double cutoffFrequencyHz)
{
   if (cutoffFrequencyHz <= 0.0)
      throw std::runtime_error("Cutoff frequency must be greater than zero. Cutoff = " + std::to_string(cutoffFrequencyHz));

   double cutoffRadPerSec = cutoffFrequencyHz * 2.0 * M_PI;
   double rc = 1.0 / cutoffRadPerSec;
   double alpha = rc / (rc + dt);

   if (alpha <= 0 || (alpha >= 1.0 && dt != 0.0))
      throw std::runtime_error("Alpha value must be between 0 and 1. Alpha = " + std::to_string(alpha));

   return alpha;
}

void FirstOrderFilteredYoDouble::reset()
{
   hasBeenCalled_ = false;
}

void FirstOrderFilteredYoDouble::setCutoffFrequencyHz(double cutoffHz)
{
   cutoffFrequencyHz_.set(cutoffHz);
}

void FirstOrderFilteredYoDouble::update(double filterInput)
{
   if (!hasBeenCalled_)
   {
      hasBeenCalled_ = true;

      filterInputOld_ = 0.0;
      filterUpdateTimeOld_ = 0.0;

      set(filterInput);
   }
   else
   {
      if (yoTime_ != nullptr)
      {
         double timeSinceLastUpdate = yoTime_->getValue() - filterUpdateTimeOld_;

         if (timeSinceLastUpdate > 0.0)
            dt_ = timeSinceLastUpdate;
         else
            reset();
      }

      double filterOutput;

      switch (filterType_)
      {
         case FirstOrderFilterType::LOW_PASS:
            filterOutput = computeLowPassUpdate(filterInput, dt_);
            break;
         case FirstOrderFilterType::HIGH_PASS:
            filterOutput = computeHighPassUpdate(filterInput, dt_);
            break;
         default:
            throw std::runtime_error("The first order filter must be either a high pass or low pass filter.");
      }

      set(filterOutput);
   }

   filterInputOld_ = filterInput;

   if (yoTime_ != nullptr)
      filterUpdateTimeOld_ = yoTime_->getValue();
}
}
