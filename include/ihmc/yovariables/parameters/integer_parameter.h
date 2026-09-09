#pragma once

#include <cstdint>
#include <memory>

#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/providers/integer_provider.h"
#include "ihmc/yovariables/variable/yo_integer.h"

namespace ihmc::yovariables::parameters
{
class IntegerParameter : public YoParameter, public providers::IntegerProvider
{
public:
   IntegerParameter(const std::string& name, registry::YoRegistry* registry);
   IntegerParameter(const std::string& name, registry::YoRegistry* registry, std::int32_t lowerBound, std::int32_t upperBound);
   IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry);
   IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int32_t lowerBound,
                     std::int32_t upperBound);
   IntegerParameter(const std::string& name, registry::YoRegistry* registry, std::int32_t initialValue);
   IntegerParameter(const std::string& name, registry::YoRegistry* registry, std::int32_t initialValue, std::int32_t lowerBound, std::int32_t upperBound);
   IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int32_t initialValue);
   IntegerParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int32_t initialValue,
                     std::int32_t lowerBound, std::int32_t upperBound);

   /** @throws exceptions::IllegalOperationException if this parameter has not been loaded. */
   std::int32_t getValue() const override;

   void setParameterBounds(std::int32_t lowerBound, std::int32_t upperBound);

   variable::YoInteger& getVariable() const override;

private:
   static constexpr std::int32_t kDefaultSuggestedMinimum = -10;
   static constexpr std::int32_t kDefaultSuggestedMaximum = 10;

   class BackingVariable : public variable::YoInteger
   {
   public:
      BackingVariable(IntegerParameter& owner, const std::string& name, const std::string& description, registry::YoRegistry* registry);

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
      IntegerParameter& owner_;
   };

   void setToDefault() override;

   std::unique_ptr<BackingVariable> value_;
   std::int32_t initialValue_;
};
}
