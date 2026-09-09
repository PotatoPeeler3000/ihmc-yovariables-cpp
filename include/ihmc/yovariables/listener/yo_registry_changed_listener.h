#pragma once

namespace ihmc::yovariables::registry
{
class YoRegistry;
}

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::listener
{
/** Receives notifications of changes to a YoRegistry. */
class YoRegistryChangedListener
{
public:
   /** A single change done to a YoRegistry. */
   class Change
   {
   public:
      virtual ~Change() = default;
      virtual bool wasRegistryAdded() const = 0;
      virtual bool wasRegistryRemoved() const = 0;
      virtual bool wasVariableAdded() const = 0;
      virtual bool wasVariableRemoved() const = 0;
      virtual bool wasCleared() const = 0;
      virtual registry::YoRegistry* getSource() const = 0;
      virtual registry::YoRegistry* getTargetParentRegistry() const = 0;
      virtual registry::YoRegistry* getTargetRegistry() const = 0;
      virtual variable::YoVariable* getTargetVariable() const = 0;
   };

   virtual ~YoRegistryChangedListener() = default;
   virtual void changed(const Change& change) = 0;
};
}
