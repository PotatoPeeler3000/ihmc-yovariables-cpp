#pragma once

#include <cstdint>
#include <memory>

#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/providers/long_provider.h"
#include "ihmc/yovariables/variable/yo_long.h"

namespace ihmc::yovariables::parameters
{
class LongParameter : public YoParameter, public providers::LongProvider
{
public:
   LongParameter(const std::string& name, registry::YoRegistry* registry);
   LongParameter(const std::string& name, registry::YoRegistry* registry, std::int64_t lowerBound, std::int64_t upperBound);
   LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry);
   LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int64_t lowerBound, std::int64_t upperBound);
   LongParameter(const std::string& name, registry::YoRegistry* registry, std::int64_t initialValue);
   LongParameter(const std::string& name, registry::YoRegistry* registry, std::int64_t initialValue, std::int64_t lowerBound, std::int64_t upperBound);
   LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int64_t initialValue);
   LongParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, std::int64_t initialValue,
                 std::int64_t lowerBound, std::int64_t upperBound);

   /** @throws exceptions::IllegalOperationException if this parameter has not been loaded. */
   std::int64_t getValue() const override;

   void setParameterBounds(std::int64_t lowerBound, std::int64_t upperBound);

   variable::YoLong& getVariable() const override;

private:
   static constexpr std::int64_t kDefaultSuggestedMinimum = -100;
   static constexpr std::int64_t kDefaultSuggestedMaximum = 100;

   class BackingVariable : public variable::YoLong
   {
   public:
      BackingVariable(LongParameter& owner, const std::string& name, const std::string& description, registry::YoRegistry* registry);

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
      LongParameter& owner_;
   };

   void setToDefault() override;

   std::unique_ptr<BackingVariable> value_;
   std::int64_t initialValue_;
};
}
