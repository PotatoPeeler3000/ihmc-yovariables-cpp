#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <magic_enum/magic_enum.hpp>

#include "ihmc/yovariables/providers/enum_provider.h"
#include "ihmc/yovariables/variable/yo_enum_holder.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::variable
{
/**
 * Enum implementation of a YoVariable.
 * <p>
 * Enum reflection (Java's Class&lt;E&gt;.getEnumConstants()/toString()) is replaced with
 * magic_enum's compile-time enum introspection - see enum_values()/enum_name() below.
 * </p>
 *
 * @tparam E the enum type used with this variable.
 */
template <typename E>
class YoEnum : public YoVariable, public providers::EnumProvider<E>, public YoEnumHolder
{
   static_assert(std::is_enum<E>::value, "E must be an enum type");

public:
   // Un-hide YoVariable's 1-arg convenience overloads: declaring the 2-arg virtuals below with the
   // same names would otherwise hide all base overloads of those names, including these.
   using YoVariable::setValueFromDouble;
   using YoVariable::setValueFromLongBits;
   using YoVariable::parseValue;

   static constexpr int NULL_VALUE = -1;
   static const std::string& nullValueString()
   {
      static const std::string value = "null";
      return value;
   }

   /** Enum-backed: initializes to the first enum constant, null value not allowed. */
   YoEnum(const std::string& name, registry::YoRegistry* registry) : YoEnum(name, "", registry, false)
   {
   }

   /** Enum-backed: initializes to the first enum constant, or null if allowNullValue. */
   YoEnum(const std::string& name, registry::YoRegistry* registry, bool allowNullValue) : YoEnum(name, "", registry, allowNullValue)
   {
   }

   /** Enum-backed: initializes to the first enum constant, or null if allowNullValue. */
   YoEnum(const std::string& name, const std::string& description, registry::YoRegistry* registry, bool allowNullValue)
      : YoVariable(YoVariableType::ENUM, name, description, registry), isBackedByEnum_(true), allowNullValue_(allowNullValue)
   {
      auto values = magic_enum::enum_values<E>();
      enumValues_.assign(values.begin(), values.end());
      enumValuesAsString_.reserve(enumValues_.size());

      for (E value : enumValues_)
      {
         std::string valueAsString(magic_enum::enum_name(value));
         if (equalsIgnoreCase(valueAsString, nullValueString()))
            throw std::invalid_argument(valueAsString + " is a restricted keyword. No enum constants named \"null\" (case insensitive) are allowed.");
         enumValuesAsString_.push_back(std::move(valueAsString));
      }

      finishConstruction();
   }

   /**
    * Not backed by an enum type: built directly from the string representation of each constant.
    * Only useful under peculiar circumstances (e.g. deserializing data that only carries the enum
    * constants' names, not the actual enum type) - the part of the API interacting with E is not
    * supported on an instance constructed this way, see isBackedByEnum().
    */
   YoEnum(const std::string& name, const std::string& description, registry::YoRegistry* registry, bool allowNullValue, std::vector<std::string> constants)
      : YoVariable(YoVariableType::ENUM, name, description, registry), isBackedByEnum_(false), allowNullValue_(allowNullValue)
   {
      for (const std::string& constant : constants)
      {
         if (equalsIgnoreCase(constant, nullValueString()))
            throw std::invalid_argument(constant + " is a restricted keyword. No enum constants named \"null\" (case insensitive) are allowed.");
      }

      enumValuesAsString_ = std::move(constants);
      finishConstruction();
   }

   /** True if this variable was constructed from a real enum type E (see isBackedByEnum()). */
   bool isBackedByEnum() const override
   {
      return isBackedByEnum_;
   }

   bool isNullAllowed() const override
   {
      return allowNullValue_;
   }

   bool valueEquals(std::optional<E> value) const
   {
      checkIfBackedByEnum();
      if (valueOrdinal_ == NULL_VALUE)
         return !value.has_value();
      return value.has_value() && static_cast<int>(*value) == valueOrdinal_;
   }

   /** @throws std::logic_error if not backed by an enum type (see isBackedByEnum()). */
   E getValue() const override
   {
      return getEnumValue();
   }

   /**
    * The current value, or std::nullopt if this variable's value is currently null.
    *
    * @throws std::logic_error if not backed by an enum type (see isBackedByEnum()).
    */
   std::optional<E> getEnumValueOrNull() const
   {
      checkIfBackedByEnum();
      return valueOrdinal_ == NULL_VALUE ? std::nullopt : std::optional<E>(enumValues_[valueOrdinal_]);
   }

   /** @throws std::logic_error if not backed by an enum type, or if the current value is null. */
   E getEnumValue() const
   {
      std::optional<E> value = getEnumValueOrNull();
      if (!value.has_value())
         throw std::logic_error("This YoEnum's current value is null.");
      return *value;
   }

   /** @throws std::logic_error if not backed by an enum type (see isBackedByEnum()). */
   const std::vector<E>& getEnumValues() const
   {
      checkIfBackedByEnum();
      return enumValues_;
   }

   const std::vector<std::string>& getEnumValuesAsString() const override
   {
      return enumValuesAsString_;
   }

   bool set(E value)
   {
      return set(std::optional<E>(value), true);
   }

   bool set(std::optional<E> value, bool notifyListeners)
   {
      checkIfBackedByEnum();

      if (!allowNullValue_ && !value.has_value())
         throw std::invalid_argument("Setting YoEnum " + getName() + " to null. Must set allowNullValue to true in the constructor if you ever want to set it to null.");

      return set(value.has_value() ? static_cast<int>(*value) : NULL_VALUE, notifyListeners);
   }

   int getOrdinal() const override
   {
      return valueOrdinal_;
   }

   bool set(int ordinal)
   {
      return set(ordinal, true);
   }

   bool set(int ordinal, bool notify)
   {
      checkBounds(ordinal);

      if (valueOrdinal_ != ordinal)
      {
         valueOrdinal_ = ordinal;
         if (notify)
            notifyListeners();
         return true;
      }

      return false;
   }

   std::string getStringValue() const
   {
      if (valueOrdinal_ == NULL_VALUE)
         return nullValueString();
      return enumValuesAsString_[valueOrdinal_];
   }

   bool setValueFromDouble(double value, bool notifyListeners) override
   {
      int ordinal = static_cast<int>(std::lround(value));
      ordinal = std::min(ordinal, getEnumSize() - 1);
      ordinal = std::max(ordinal, allowNullValue_ ? NULL_VALUE : 0);
      return set(ordinal, notifyListeners);
   }

   double getValueAsDouble() const override
   {
      return valueOrdinal_;
   }

   std::int64_t getValueAsLongBits() const override
   {
      return valueOrdinal_;
   }

   bool setValueFromLongBits(std::int64_t value, bool notifyListeners) override
   {
      return set(static_cast<int>(value), notifyListeners);
   }

   bool setValue(YoVariable& other, bool notifyListeners) override
   {
      auto& otherEnum = static_cast<YoEnum<E>&>(other);
      if (otherEnum.isBackedByEnum() && isBackedByEnum())
         return set(otherEnum.getEnumValueOrNull(), notifyListeners);
      return set(otherEnum.getOrdinal(), notifyListeners);
   }

   int getEnumSize() const
   {
      return static_cast<int>(enumValuesAsString_.size());
   }

   std::string getValueAsString(const std::optional<std::string>&) const override
   {
      return getStringValue();
   }

   bool parseValue(const std::string& valueAsString, bool notifyListeners) override
   {
      std::string lower = valueAsString;
      std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
      if (lower == nullValueString())
         return set(NULL_VALUE, notifyListeners);

      for (std::size_t i = 0; i < enumValuesAsString_.size(); i++)
      {
         if (valueAsString == enumValuesAsString_[i])
            return set(static_cast<int>(i), notifyListeners);
      }

      throw std::invalid_argument("Unable to parse value for parameter: " + getFullNameString() + ". String value: " + valueAsString);
   }

   std::string convertDoubleValueToString(const std::optional<std::string>&, double value) const override
   {
      int ordinal = static_cast<int>(std::lround(value));
      ordinal = std::min(ordinal, getEnumSize() - 1);
      ordinal = std::max(ordinal, allowNullValue_ ? NULL_VALUE : 0);
      if (ordinal == NULL_VALUE)
         return nullValueString();
      return enumValuesAsString_[ordinal];
   }

   bool isZero() const override
   {
      return valueOrdinal_ == NULL_VALUE;
   }

   std::unique_ptr<YoVariable> duplicate(registry::YoRegistry* newRegistry) const override
   {
      std::unique_ptr<YoEnum<E>> duplicate;
      if (isBackedByEnum())
         duplicate = std::make_unique<YoEnum<E>>(getName(), getDescription(), newRegistry, isNullAllowed());
      else
         duplicate = std::make_unique<YoEnum<E>>(getName(), getDescription(), newRegistry, isNullAllowed(), enumValuesAsString_);
      duplicate->set(getOrdinal());
      return duplicate;
   }

   std::string toString() const override
   {
      return getName() + ": " + getStringValue();
   }

private:
   void finishConstruction()
   {
      if (!allowNullValue_ && enumValuesAsString_.empty())
         throw std::invalid_argument("Cannot initialize an enum variable with zero elements if allowNullValue is false.");

      if (allowNullValue_ || enumValuesAsString_.empty())
         set(NULL_VALUE);
      else
         set(0);

      setVariableBounds(allowNullValue_ ? NULL_VALUE : 0, static_cast<double>(enumValuesAsString_.size()) - 1.0);
   }

   void checkIfBackedByEnum() const
   {
      if (!isBackedByEnum_)
         throw std::logic_error("This YoEnum is not backed by an Enum type.");
   }

   void checkBounds(int ordinal) const
   {
      if ((ordinal < 0 && !(allowNullValue_ && ordinal == NULL_VALUE)) || ordinal >= static_cast<int>(enumValuesAsString_.size()))
         throw std::runtime_error("Enum constant associated with value " + std::to_string(ordinal) + " not present.");
   }

   static bool equalsIgnoreCase(const std::string& a, const std::string& b)
   {
      if (a.size() != b.size())
         return false;
      return std::equal(a.begin(), a.end(), b.begin(), [](unsigned char c1, unsigned char c2) { return std::tolower(c1) == std::tolower(c2); });
   }

   bool isBackedByEnum_;
   bool allowNullValue_;
   std::vector<E> enumValues_;
   std::vector<std::string> enumValuesAsString_;
   int valueOrdinal_ = 0;
};
}
