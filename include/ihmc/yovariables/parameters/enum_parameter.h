#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <magic_enum/magic_enum.hpp>

#include "ihmc/yovariables/parameters/yo_parameter.h"
#include "ihmc/yovariables/providers/enum_provider.h"
#include "ihmc/yovariables/variable/yo_enum.h"

namespace ihmc::yovariables::parameters
{
/**
 * Enum parameter.
 *
 * @tparam E the enum type used with this parameter.
 */
template <typename E>
class EnumParameter : public YoParameter, public providers::EnumProvider<E>
{
public:
   /** Enum-backed: initial value is the first enum constant, or null if allowNullValue. */
   EnumParameter(const std::string& name, registry::YoRegistry* registry, bool allowNullValue)
      : EnumParameter(name, "", registry, allowNullValue)
   {
   }

   /** Enum-backed: initial value is the first enum constant, or null if allowNullValue. */
   EnumParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, bool allowNullValue)
      : EnumParameter(name,
                       description,
                       registry,
                       allowNullValue,
                       allowNullValue ? std::optional<E>(std::nullopt) : std::optional<E>(magic_enum::enum_values<E>().front()))
   {
   }

   /** Enum-backed, with an explicit initial value. */
   EnumParameter(const std::string& name, registry::YoRegistry* registry, bool allowNullValue, std::optional<E> initialValue)
      : EnumParameter(name, "", registry, allowNullValue, initialValue)
   {
   }

   /** Enum-backed, with an explicit initial value. */
   EnumParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, bool allowNullValue,
                 std::optional<E> initialValue)
      : value_(std::make_unique<BackingVariable>(*this, name, description, registry, allowNullValue))
   {
      if (!value_->isNullAllowed() && !initialValue.has_value())
         throw std::invalid_argument("Cannot initialize to null value, allowNullValue is false");

      initialOrdinal_ = initialValue.has_value() ? static_cast<int>(*initialValue) : variable::YoEnum<E>::NULL_VALUE;
   }

   /**
    * Not backed by an enum type: built directly from the string representation of each constant.
    * See YoEnum's equivalent constructor - only useful under peculiar circumstances (e.g.
    * deserializing data that only carries the enum constants' names).
    */
   EnumParameter(const std::string& name, const std::string& description, registry::YoRegistry* registry, bool allowNullValues,
                 std::vector<std::string> constants)
      : value_(std::make_unique<BackingVariable>(*this, name, description, registry, allowNullValues, std::move(constants)))
   {
      initialOrdinal_ = allowNullValues || value_->getEnumSize() == 0 ? variable::YoEnum<E>::NULL_VALUE : 0;
   }

   /** @throws exceptions::IllegalOperationException if this parameter has not been loaded. */
   E getValue() const override
   {
      checkLoaded();
      return value_->getEnumValue();
   }

   bool isBackedByEnum() const
   {
      return value_->isBackedByEnum();
   }

   bool isNullAllowed() const
   {
      return value_->isNullAllowed();
   }

   const std::vector<E>& getEnumValues() const
   {
      return value_->getEnumValues();
   }

   const std::vector<std::string>& getEnumValuesAsString() const
   {
      return value_->getEnumValuesAsString();
   }

   int getEnumSize() const
   {
      return value_->getEnumSize();
   }

   /** Matches the Java source: returns the base YoVariable type, not YoEnum<E>. */
   variable::YoVariable& getVariable() const override
   {
      return *value_;
   }

private:
   class BackingVariable : public variable::YoEnum<E>
   {
   public:
      BackingVariable(EnumParameter<E>& owner, const std::string& name, const std::string& description, registry::YoRegistry* registry,
                       bool allowNullValue)
         : variable::YoEnum<E>(name, description, nullptr, allowNullValue), owner_(owner)
      {
         // Registration is deferred to here (the constructor body), after the base subobject is
         // fully constructed: see BooleanParameter::BackingVariable::BackingVariable for why -
         // registering earlier would resolve isParameter()/getParameter() to YoVariable's defaults
         // rather than this class's overrides, since C++ (unlike Java) dispatches virtual calls using
         // the under-construction base's own vtable while a base class subobject is being built.
         this->setRegistry(registry);
      }

      BackingVariable(EnumParameter<E>& owner, const std::string& name, const std::string& description, registry::YoRegistry* registry,
                       bool allowNullValue, std::vector<std::string> constants)
         : variable::YoEnum<E>(name, description, nullptr, allowNullValue, std::move(constants)), owner_(owner)
      {
         this->setRegistry(registry);
      }

      bool isParameter() const override
      {
         return true;
      }

      YoParameter* getParameter() const override
      {
         return &owner_;
      }

      std::unique_ptr<variable::YoVariable> duplicate(registry::YoRegistry* newRegistry) const override
      {
         // Deliberately leaked - see BooleanParameter::BackingVariable::duplicate for why.
         EnumParameter<E>* newParameter;

         if (this->isBackedByEnum())
         {
            std::optional<E> initialValue = owner_.initialOrdinal_ == variable::YoEnum<E>::NULL_VALUE
                                                ? std::optional<E>(std::nullopt)
                                                : std::optional<E>(this->getEnumValues()[owner_.initialOrdinal_]);
            newParameter = new EnumParameter<E>(this->getName(), this->getDescription(), newRegistry, this->isNullAllowed(), initialValue);
         }
         else
         {
            newParameter = new EnumParameter<E>(this->getName(), this->getDescription(), newRegistry, this->isNullAllowed(), this->getEnumValuesAsString());
         }

         newParameter->value_->set(this->getOrdinal());
         newParameter->setLoadStatus(owner_.getLoadStatus());
         return std::unique_ptr<variable::YoVariable>(newParameter->value_.get());
      }

   private:
      EnumParameter<E>& owner_;
   };

   void setToDefault() override
   {
      value_->set(initialOrdinal_);
   }

   std::unique_ptr<BackingVariable> value_;
   int initialOrdinal_ = variable::YoEnum<E>::NULL_VALUE;
};
}
