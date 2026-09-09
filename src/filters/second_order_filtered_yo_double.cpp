#include "ihmc/yovariables/filters/second_order_filtered_yo_double.h"

#include <cmath>
#include <stdexcept>

namespace ihmc::yovariables::filters
{
namespace
{
std::vector<std::unique_ptr<variable::YoDouble>> makeThree(const std::string& baseName, registry::YoRegistry* registry)
{
   std::vector<std::unique_ptr<variable::YoDouble>> result;
   result.reserve(3);
   for (int i = 0; i < 3; i++)
      result.push_back(std::make_unique<variable::YoDouble>(baseName + std::to_string(i), registry));
   return result;
}
} // namespace

SecondOrderFilteredYoDouble::SecondOrderFilteredYoDouble(const std::string& name, registry::YoRegistry* registry, double dt,
                                                           double naturalFrequencyInHz, double dampingRatio, SecondOrderFilterType filterType,
                                                           variable::YoDouble* inputVariable)
   : YoDouble(name, registry), dt_(dt), hasBeenCalled_(name + "HasBeenCalled", registry), inputVariable_(inputVariable),
     input_(makeThree(name + "input", registry)), output_(makeThree(name + "output", registry))
{
   internalParameters_.emplace(name, registry, naturalFrequencyInHz, dampingRatio, filterType);
   parameters_ = &*internalParameters_;
   reset();
}

SecondOrderFilteredYoDouble::SecondOrderFilteredYoDouble(const std::string& name, registry::YoRegistry* registry, double dt,
                                                           SecondOrderFilteredYoVariableParameters& parameters, variable::YoDouble* inputVariable)
   : YoDouble(name, registry), dt_(dt), parameters_(&parameters), hasBeenCalled_(name + "HasBeenCalled", registry), inputVariable_(inputVariable),
     input_(makeThree(name + "input", registry)), output_(makeThree(name + "output", registry))
{
   reset();
}

void SecondOrderFilteredYoDouble::reset()
{
   hasBeenCalled_.set(false);
   computeCoefficients();
}

void SecondOrderFilteredYoDouble::update()
{
   if (inputVariable_ == nullptr)
      throw std::logic_error("SecondOrderFilteredYoDouble must be constructed with a non-null input variable to call update().");
   update(inputVariable_->getDoubleValue());
}

void SecondOrderFilteredYoDouble::update(double currentInputValue)
{
   if (!hasBeenCalled_.getBooleanValue())
   {
      hasBeenCalled_.set(true);
      set(currentInputValue);
      for (int i = 0; i < 3; i++)
      {
         input_[static_cast<std::size_t>(i)]->set(currentInputValue);
         output_[static_cast<std::size_t>(i)]->set(currentInputValue);
      }
      return;
   }

   for (int i = 2; i > 0; i--)
   {
      input_[static_cast<std::size_t>(i)]->set(input_[static_cast<std::size_t>(i - 1)]->getDoubleValue());
      output_[static_cast<std::size_t>(i)]->set(output_[static_cast<std::size_t>(i - 1)]->getDoubleValue());
   }
   input_[0]->set(currentInputValue);

   double currentOutputValue = 0.0;
   currentOutputValue += b_[2] * input_[2]->getDoubleValue();
   currentOutputValue += b_[1] * input_[1]->getDoubleValue();
   currentOutputValue += b_[0] * input_[0]->getDoubleValue();
   currentOutputValue -= a_[2] * output_[2]->getDoubleValue();
   currentOutputValue -= a_[1] * output_[1]->getDoubleValue();
   currentOutputValue /= a_[0];
   output_[0]->set(currentOutputValue);

   set(currentOutputValue);
}

void SecondOrderFilteredYoDouble::setNaturalFrequencyInHz(double naturalFrequencyInHz)
{
   parameters_->getNaturalFrequencyInHz().set(std::min(std::max(naturalFrequencyInHz, 0.0), 1.0 / (2.0 * dt_)));
   computeCoefficients();
}

void SecondOrderFilteredYoDouble::setDampingRatio(double dampingRatio)
{
   parameters_->getDampingRatio().set(std::max(dampingRatio, 0.0));
   computeCoefficients();
}

bool SecondOrderFilteredYoDouble::getHasBeenCalled() const
{
   return hasBeenCalled_.getBooleanValue();
}

void SecondOrderFilteredYoDouble::getFilterCoefficients(std::vector<double>& b, std::vector<double>& a) const
{
   if (b.size() < 3)
      throw std::runtime_error("b must be of length 3 or greater");
   if (a.size() < 3)
      throw std::runtime_error("a must be of length 3 or greater");

   for (int i = 0; i < 3; i++)
      b[static_cast<std::size_t>(i)] = b_[static_cast<std::size_t>(i)];
   for (std::size_t i = 3; i < b.size(); i++)
      b[i] = 0.0;
   for (int i = 0; i < 3; i++)
      a[static_cast<std::size_t>(i)] = a_[static_cast<std::size_t>(i)];
   for (std::size_t i = 3; i < a.size(); i++)
      a[i] = 0.0;
}

void SecondOrderFilteredYoDouble::computeCoefficients()
{
   double omega = 2 * M_PI * parameters_->getNaturalFrequencyInHz().getDoubleValue();
   double xi = parameters_->getDampingRatio().getDoubleValue();

   switch (parameters_->getFilterType())
   {
      case SecondOrderFilterType::LOW_PASS:
         b_[0] = omega * omega;
         b_[1] = 2.0 * omega * omega;
         b_[2] = omega * omega;
         break;
      case SecondOrderFilterType::NOTCH:
         b_[0] = 4.0 / (dt_ * dt_) + omega * omega;
         b_[1] = 2.0 * omega * omega - 8.0 / (dt_ * dt_);
         b_[2] = 4.0 / (dt_ * dt_) + omega * omega;
         break;
      case SecondOrderFilterType::HIGH_PASS:
         b_[0] = 4.0 / (dt_ * dt_);
         b_[1] = -8.0 / (dt_ * dt_);
         b_[2] = 4.0 / (dt_ * dt_);
         break;
      case SecondOrderFilterType::BAND:
         throw std::invalid_argument("Band pass filters are not established for the second order filter yo variable.");
   }

   a_[0] = 4.0 / (dt_ * dt_) + 4.0 / dt_ * xi * omega + omega * omega;
   a_[1] = 2.0 * omega * omega - 8.0 / (dt_ * dt_);
   a_[2] = 4.0 / (dt_ * dt_) - 4.0 / dt_ * xi * omega + omega * omega;
}
}
