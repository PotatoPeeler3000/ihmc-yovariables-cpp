#pragma once

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "ihmc/yovariables/filters/processing_yo_variable.h"
#include "ihmc/yovariables/filters/second_order_filtered_yo_variable_parameters.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
/**
 * A discrete-time second order filter using the bilinear transform. See
 * SecondOrderFilterType for the supported transfer functions (LOW_PASS/NOTCH/HIGH_PASS; BAND is
 * declared but not implemented, matching the Java source).
 */
class SecondOrderFilteredYoDouble : public variable::YoDouble, public ProcessingYoVariable
{
public:
   SecondOrderFilteredYoDouble(const std::string& name, registry::YoRegistry* registry, double dt, double naturalFrequencyInHz, double dampingRatio,
                                SecondOrderFilterType filterType, variable::YoDouble* inputVariable = nullptr);
   SecondOrderFilteredYoDouble(const std::string& name, registry::YoRegistry* registry, double dt, SecondOrderFilteredYoVariableParameters& parameters,
                                variable::YoDouble* inputVariable = nullptr);

   void reset() override;

   /** @throws std::logic_error if constructed without an input variable. */
   void update() override;
   void update(double currentInputValue);

   void setNaturalFrequencyInHz(double naturalFrequencyInHz);
   void setDampingRatio(double dampingRatio);

   bool getHasBeenCalled() const;

   /** @throws std::runtime_error if b or a has fewer than 3 elements. */
   void getFilterCoefficients(std::vector<double>& b, std::vector<double>& a) const;

private:
   void computeCoefficients();

   double dt_;
   SecondOrderFilteredYoVariableParameters* parameters_;
   variable::YoBoolean hasBeenCalled_;
   variable::YoDouble* inputVariable_ = nullptr;
   std::vector<std::unique_ptr<variable::YoDouble>> input_;
   std::vector<std::unique_ptr<variable::YoDouble>> output_;
   std::array<double, 3> a_{};
   std::array<double, 3> b_{};

   std::optional<SecondOrderFilteredYoVariableParameters> internalParameters_;
};
}
