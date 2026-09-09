#pragma once

#include <limits>
#include <memory>

#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/providers/double_provider.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::parameters
{
class DoubleParameter : public YoParameter, public providers::DoubleProvider
{
public:
   DoubleParameter(const std::string& name, registry::YoRegistry* registry);
   DoubleParameter(const std::string& name, registry::YoRegistry* registry, double lowerBound, double upperBound);
   DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry);
   DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, double lowerBound, double upperBound);
   DoubleParameter(const std::string& name, registry::YoRegistry* registry, double initialValue);
   DoubleParameter(const std::string& name, registry::YoRegistry* registry, double initialValue, double lowerBound, double upperBound);
   DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, double initialValue);
   DoubleParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, double initialValue, double lowerBound,
                    double upperBound);

   /** @throws exceptions::IllegalOperationException if this parameter has not been loaded. */
   double getValue() const override;

   void setParameterBounds(double lowerBound, double upperBound);

   variable::YoDouble& getVariable() const override;

private:
   static constexpr double kDefaultSuggestedMinimum = 0.0;
   static constexpr double kDefaultSuggestedMaximum = 1.0;

   class BackingVariable : public variable::YoDouble
   {
   public:
      BackingVariable(DoubleParameter& owner, const std::string& name, const std::string& description, registry::YoRegistry* registry);

      bool isParameter() const override
      {
         return true;
      }

      YoParameter* getParameter() const override
      {
         return &owner_;
      }

      std::unique_ptr<variable::YoVariable> duplicate(registry::YoRegistry* newRegistry) const override;

   private:
      DoubleParameter& owner_;
   };

   void setToDefault() override;

   std::unique_ptr<BackingVariable> value_;
   double initialValue_;
};
}
