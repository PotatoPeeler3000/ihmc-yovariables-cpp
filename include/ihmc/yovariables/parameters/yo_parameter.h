#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ihmc/yovariables/parameters/parameter_load_status.h"

namespace ihmc::yovariables::listener
{
class YoParameterChangedListener;
}

namespace ihmc::yovariables::registry
{
class YoNamespace;
}

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::parameters
{
/**
 * Base class for parameters.
 * <p>
 * A parameter can be seen as a read-only YoVariable. The intention for parameters is the guarantee
 * of not being modified by the algorithm/controller/module that uses the parameter's value for
 * computation - only whatever loads/writes parameters (e.g. an XmlParameterReader, or a GUI) can.
 * </p>
 * <p>
 * Parameters can be initialized only once and have to be initialized before they can be used, via
 * a parameter reader (see AbstractParameterReader). Two default implementations are provided in
 * this package: DefaultParameterReader (initializes to each parameter's constructor-provided
 * value) and XmlParameterReader (parses initial values from an XML file).
 * </p>
 */
class YoParameter
{
public:
   /**
    * Declared (not = default) and defined in the .cpp: the implicit destructor would need
    * ChangedListenerHolder's complete type wherever this header is included, but that type is only
    * forward-declared here.
    */
   virtual ~YoParameter();

   std::string getName() const;
   std::string getDescription() const;
   const registry::YoNamespace& getFullName() const;
   std::string getFullNameString() const;
   const registry::YoNamespace* getNamespace() const;

   double getLowerBound() const;
   double getUpperBound() const;

   void addListener(listener::YoParameterChangedListener* listenerToAdd);
   void removeListeners();
   const std::vector<listener::YoParameterChangedListener*>& getListeners() const;
   bool removeListener(listener::YoParameterChangedListener* listenerToRemove);

   /** @throws exceptions::IllegalOperationException if this parameter has not been loaded. */
   std::string getValueAsString();

   /**
    * The variable backing this parameter. Public (unlike the Java source's package-private
    * equivalent, relaxed since C++ has no package-private access level) since it is used by parameter
    * readers/writers in this package. const-qualified for logical constness only: it never mutates
    * this parameter itself, though the returned reference is mutable, matching the fact that loading
    * a parameter's value is exactly the kind of "not logically part of YoParameter's own state"
    * mutation that goes through the backing variable.
    */
   virtual variable::YoVariable& getVariable() const = 0;

   /** Tries to parse valueAsString and set this parameter's value; typically used to initialize it. */
   void setToString(const std::string& valueString);

   /** Sets this parameter's value to the initial value provided at construction. */
   virtual void setToDefault() = 0;

   void load(const std::string& valueString);
   void loadDefault();

   ParameterLoadStatus getLoadStatus() const;

   /**
    * Sets the load status directly, bypassing setToString/setToDefault. Public (unlike the Java
    * source's package-private field write - see getVariable()) since SingleParameterReader, in this
    * same package, needs it when initializing a parameter from an already-known value.
    */
   void setLoadStatus(ParameterLoadStatus status);

   /** @throws exceptions::IllegalOperationException if this parameter has not been loaded. */
   void checkLoaded() const;

   bool isLoaded() const;

   std::string toString();

   /**
    * Sets the bounds for this parameter's range of values (not enforced; used for GUI purposes).
    * Public (unlike the Java source's package-private equivalent - see getVariable()) since
    * ParameterData, in this same package, needs to call it when loading a parameter's value.
    */
   void setParameterBounds(double lowerBound, double upperBound);

protected:
   /** Initialization state; settable by subclasses (e.g. when cloning into a fresh parameter). */
   ParameterLoadStatus loadStatus_ = ParameterLoadStatus::UNLOADED;

private:
   class ChangedListenerHolder;

   /**
    * Raw owning pointer rather than std::unique_ptr<ChangedListenerHolder>: libc++'s unique_ptr
    * destructor is eagerly instantiated for constexpr-eligibility checking even when never called,
    * which requires ChangedListenerHolder's complete type wherever this header is included. A raw
    * pointer only needs completeness at the actual `delete`, in yo_parameter.cpp.
    */
   ChangedListenerHolder* changedListenerHolder_ = nullptr;
};
}
