#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/variable/yo_variable_type.h"

namespace ihmc::yovariables::registry
{
class YoRegistry;
}

namespace ihmc::yovariables::parameters
{
class YoParameter;
}

namespace ihmc::yovariables::listener
{
class YoVariableChangedListener;
}

namespace ihmc::yovariables::variable
{
/**
 * Base for a framework that allows storing, manipulating, logging, and visualizing data.
 * <p>
 * Ownership: a YoVariable does not own, and is not owned by, its YoRegistry - it stores a
 * non-owning back-pointer, mirroring the Java library's reference semantics. Whoever constructs a
 * YoVariable (e.g. a controller class holding it as a member) is responsible for its lifetime.
 * </p>
 */
class YoVariable
{
public:
   YoVariable(YoVariableType type, const std::string& name, const std::string& description, registry::YoRegistry* registry);
   virtual ~YoVariable() = default;

   YoVariable(const YoVariable&) = delete;
   YoVariable& operator=(const YoVariable&) = delete;

   /** Sets the registry for this variable. Pass nullptr to detach. */
   void setRegistry(registry::YoRegistry* registry);

   /** Forces getFullName() to recompute its cache on next call. For internal use. */
   void resetFullName();

   registry::YoRegistry* getRegistry() const;

   const std::string& getName() const;

   /** This variable's description, or "" if not specified. */
   const std::string& getDescription() const;

   /** This variable's name prepended with its parent registry's namespace, as a namespace. */
   const registry::YoNamespace& getFullName();

   std::string getFullNameString();

   /** The namespace of the registry this variable is registered to, or nullptr if unregistered. */
   const registry::YoNamespace* getNamespace() const;

   void setVariableBounds(double lowerBound, double upperBound);
   double getLowerBound() const;
   double getUpperBound() const;

   YoVariableType getType() const;

   void addListener(listener::YoVariableChangedListener* listenerToAdd);
   void removeListeners();
   const std::vector<listener::YoVariableChangedListener*>& getListeners() const;
   bool removeListener(listener::YoVariableChangedListener* listenerToRemove);
   void notifyListeners();

   virtual double getValueAsDouble() const = 0;
   bool setValueFromDouble(double value);
   virtual bool setValueFromDouble(double value, bool notifyListeners) = 0;

   virtual std::int64_t getValueAsLongBits() const = 0;
   bool setValueFromLongBits(std::int64_t value);
   virtual bool setValueFromLongBits(std::int64_t value, bool notifyListeners) = 0;

   /** Sets this variable's value from other, which must be the same concrete type as this. */
   virtual bool setValue(YoVariable& other, bool notifyListeners) = 0;

   std::string getValueAsString();
   virtual std::string getValueAsString(const std::optional<std::string>& format) const = 0;

   bool parseValue(const std::string& valueAsString);
   virtual bool parseValue(const std::string& valueAsString, bool notifyListeners) = 0;

   virtual std::string convertDoubleValueToString(const std::optional<std::string>& format, double value) const = 0;

   virtual bool isZero() const = 0;

   /** True if this variable should be treated as a parameter. */
   virtual bool isParameter() const
   {
      return false;
   }

   /** If isParameter() is true, the corresponding parameter; nullptr otherwise. */
   virtual parameters::YoParameter* getParameter() const
   {
      return nullptr;
   }

   /** Detaches this variable from its parent and clears its listeners. */
   virtual void destroy();

   /**
    * Creates a new variable of this same concrete type and current value, registered to
    * newRegistry. Ownership of the returned variable transfers to the caller (unlike this
    * variable itself, which the registry never owns) since it is a freshly allocated copy with no
    * other owner.
    */
   virtual std::unique_ptr<YoVariable> duplicate(registry::YoRegistry* newRegistry) const = 0;

   virtual std::string toString() const = 0;

private:
   std::string name_;
   std::string description_;
   YoVariableType type_;
   registry::YoRegistry* registry_ = nullptr;
   std::optional<registry::YoNamespace> fullName_;

   std::vector<listener::YoVariableChangedListener*> changedListeners_;
   double lowerBound_ = 0.0;
   double upperBound_ = 1.0;
};
}
