#pragma once

#include <memory>

#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/providers/boolean_provider.h"
#include "ihmc/yovariables/variable/yo_boolean.h"

namespace ihmc::yovariables::parameters
{
class BooleanParameter : public YoParameter, public providers::BooleanProvider
{
public:
   BooleanParameter(const std::string& name, registry::YoRegistry* registry);
   BooleanParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry);
   BooleanParameter(const std::string& name, registry::YoRegistry* registry, bool initialValue);
   BooleanParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, bool initialValue);

   /** @throws exceptions::IllegalOperationException if this parameter has not been loaded. */
   bool getValue() const override;

   variable::YoBoolean& getVariable() const override;

private:
   /**
    * Backs this parameter; the Java source's private inner class becomes an explicit back-pointer.
    * Heap-owned (rather than a value member) specifically so duplicate() can hand the caller a
    * genuinely independently-owned YoVariable: extracting a value-member subobject into a
    * unique_ptr would be undefined behavior once that unique_ptr tried to delete it.
    */
   class BackingVariable : public variable::YoBoolean
   {
   public:
      BackingVariable(BooleanParameter& owner, const std::string& name, const std::string& description, registry::YoRegistry* registry);

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
      BooleanParameter& owner_;
   };

   void setToDefault() override;

   std::unique_ptr<BackingVariable> value_;
   bool initialValue_;
};
}
