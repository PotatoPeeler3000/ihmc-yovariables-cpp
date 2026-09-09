#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ihmc/yovariables/listener/yo_registry_changed_listener.h"
#include "ihmc/yovariables/registry/yo_namespace.h"
#include "ihmc/yovariables/registry/yo_registry_restriction_level.h"
#include "ihmc/yovariables/registry/yo_variable_holder.h"

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::parameters
{
class YoParameter;
}

namespace ihmc::yovariables::registry
{
/**
 * Data structure for creating, managing, and interacting with a hierarchy of YoVariables.
 * <p>
 * Registries are organized into a tree structure with one parent, variables, and child
 * registries. Each YoVariable can only be registered to one registry at a time.
 * </p>
 * <p>
 * Ownership: mirrors the Java library's reference semantics rather than introducing C++-style
 * ownership. A YoRegistry does not own its variables or child registries (it stores non-owning
 * pointers) - the caller that constructed a YoVariable or YoRegistry is responsible for its
 * lifetime, exactly as in the real usage pattern where a controller class holds its YoDoubles
 * etc. as long-lived members and only registers them here.
 * </p>
 */
class YoRegistry : public YoVariableHolder
{
public:
   explicit YoRegistry(const std::string& name);

   const std::string& getName() const;

   const YoNamespace& getNamespace() const;

   /**
    * Raises this registry's restriction level (can only become more restrictive) and propagates
    * the change down to any descendant with a lower restriction level.
    */
   void setRestrictionLevel(YoRegistryRestrictionLevel restrictionLevel);

   YoRegistryRestrictionLevel getRestrictionLevel() const;

   /**
    * Detaches this registry from its parent, destroys every variable and child registry in its
    * subtree, and reverts the restriction level to FULLY_MUTABLE.
    *
    * @throws exceptions::IllegalOperationException if this registry is not the root and is not
    *         fully mutable.
    */
   void destroy();

   void addListener(listener::YoRegistryChangedListener* listenerToAdd);
   void removeListeners();
   bool removeListener(listener::YoRegistryChangedListener* listenerToRemove);

   /**
    * Registers variable in this registry. Does nothing if already registered here.
    *
    * @throws exceptions::NameCollisionException if variable's name collides with another variable
    *         already registered here.
    * @throws exceptions::IllegalOperationException if not permitted by the restriction level.
    */
   void addVariable(variable::YoVariable* variable);

   void removeVariable(variable::YoVariable* variable);

   void addChild(YoRegistry* child, bool notifyListeners = true);
   void removeChild(YoRegistry* child);
   void detachFromParent();

   bool hasParameters() const;
   bool hasParametersDeep() const;

   bool isRoot() const;
   YoRegistry* getRoot();
   YoRegistry* getParent();

   YoRegistry* getChild(const std::string& name) const;

   /** This registry's direct children, typed as YoRegistry (see YoVariableHolder::getChildren). */
   const std::vector<YoRegistry*>& getChildRegistries() const;

   variable::YoVariable* getVariable(const std::string& name) const;

   /** The variable at the given index, in the order variables were registered. */
   variable::YoVariable* getVariable(std::size_t index) const;

   parameters::YoParameter* getParameter(const std::string& name) const;
   const std::vector<parameters::YoParameter*>& getParameters() const;

   std::vector<variable::YoVariable*> collectSubtreeVariables() const;
   std::vector<parameters::YoParameter*> collectSubtreeParameters() const;
   std::vector<YoRegistry*> collectSubtreeRegistries();

   YoRegistry* findRegistry(const std::string& name);
   YoRegistry* findRegistry(const std::optional<std::string>& namespaceEnding, const std::string& name);
   YoRegistry* findRegistry(const YoNamespace& namespaceEnding);

   std::size_t getNumberOfVariables() const;
   std::size_t getNumberOfVariablesDeep() const;

   // YoVariableHolder:
   std::vector<variable::YoVariable*> getVariables() override;
   std::vector<YoVariableHolder*> getChildren() override;
   variable::YoVariable* findVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) override;
   std::vector<variable::YoVariable*> findVariables(const std::optional<std::string>& namespaceEnding, const std::string& name) override;
   std::vector<variable::YoVariable*> findVariables(const YoNamespace& namespaceValue) override;
   bool hasUniqueVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) override;

   using YoVariableHolder::findVariable;
   using YoVariableHolder::findVariables;
   using YoVariableHolder::hasUniqueVariable;
   using YoVariableHolder::hasVariable;

   bool operator==(const YoRegistry& other) const;
   std::string toString() const;

private:
   void destroyInternal(bool clearListeners);
   void setParentNamespace(const YoNamespace* parentNamespace);
   std::size_t countNumberOfVariables(const std::optional<std::string>& parentNamespace, const std::string& name) const;

   enum class ChangeType
   {
      REGISTRY_ADDED,
      REGISTRY_REMOVED,
      VARIABLE_ADDED,
      VARIABLE_REMOVED,
      CLEARED
   };
   void notifyListeners(YoRegistry* targetParentRegistry, YoRegistry* targetRegistry, variable::YoVariable* targetVariable, ChangeType type);

   class RegistryChange;

   std::string name_;
   YoNamespace namespace_;

   std::vector<variable::YoVariable*> variables_;
   std::unordered_map<std::string, variable::YoVariable*> nameToVariableMap_;
   std::vector<parameters::YoParameter*> parameters_;

   YoRegistry* parent_ = nullptr;
   std::vector<YoRegistry*> children_;
   std::unordered_map<std::string, YoRegistry*> nameToChildMap_;

   std::vector<listener::YoRegistryChangedListener*> changedListeners_;

   YoRegistryRestrictionLevel restrictionLevel_ = YoRegistryRestrictionLevel::FULLY_MUTABLE;
};
}
