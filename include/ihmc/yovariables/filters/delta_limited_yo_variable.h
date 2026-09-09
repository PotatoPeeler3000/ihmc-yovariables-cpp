#pragma once

#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
/** Guarantees its value stays within maxDelta of the desired value passed to updateOutput(). */
class DeltaLimitedYoVariable : public variable::YoDouble
{
public:
   DeltaLimitedYoVariable(const std::string& name, registry::YoRegistry* registry, double maxDelta);

   void setMaxDelta(double maxDelta);
   void updateOutput(double actual, double desired);
   bool isLimitingActive() const;

private:
   void updateOutput();

   variable::YoDouble maxDelta_;
   variable::YoDouble actual_;
   variable::YoDouble desired_;
   variable::YoBoolean isLimitingActive_;
};
}
