#pragma once

#include <optional>

#include "ihmc/yovariables/filters/processing_yo_variable.h"
#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
/**
 * A filtered version of an input signal: filtered_n = alpha * filtered_{n-1} + (1 - alpha) * raw_n.
 * alpha=0 -> no filtering, alpha=1 -> value never changes (raw signal ignored).
 * <p>
 * Either construct with a position variable to track and call update() every tick, or call
 * update(double) every tick directly.
 * </p>
 */
class AlphaFilteredYoVariable : public variable::YoDouble, public ProcessingYoVariable
{
public:
   AlphaFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, double alpha, variable::YoDouble* positionVariable = nullptr);
   AlphaFilteredYoVariable(const std::string& name, registry::YoRegistry* registry, providers::DoubleProvider& alphaVariable,
                            variable::YoDouble* positionVariable = nullptr);
   AlphaFilteredYoVariable(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                            providers::DoubleProvider& alphaVariable, variable::YoDouble* positionVariable = nullptr);

   void reset() override;

   /** @throws std::logic_error if constructed without a position variable. */
   void update() override;
   virtual void update(double currentPosition);

   bool getHasBeenCalled() const;

protected:
   variable::YoBoolean hasBeenCalled;

private:
   providers::DoubleProvider* alphaVariable_ = nullptr;
   variable::YoDouble* position_ = nullptr;
   std::optional<variable::YoDouble> internalAlpha_;
};
}
