#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ihmc/yovariables/registry/yo_variable_holder.h"

namespace ihmc::yovariables::variable
{
class YoVariable;
}

namespace ihmc::yovariables::registry
{
/**
 * A data structure storing a collection of variables (as non-owning pointers, mirroring
 * YoRegistry's ownership model - see YoRegistry's class comment) while providing an interface to
 * facilitate queries using variable name or namespace.
 * <p>
 * Not part of the original port's earlier phases: this class was missed by Phases 1-5 (it lives in
 * the same Java package as YoRegistry but wasn't among the classes carried over) and was only
 * discovered while mechanically porting YoVariableListTest in Phase 6. Ported here as production
 * code, not just a test fixture, once the gap was found.
 * </p>
 * <p>
 * Index parameters use a signed int, rather than this port's usual std::size_t, specifically so
 * that a negative index reproduces Java's IndexOutOfBoundsException-on-get(-1) behavior rather than
 * wrapping around to a huge unsigned value.
 * </p>
 */
class YoVariableList : public YoVariableHolder
{
public:
   explicit YoVariableList(std::string name);
   YoVariableList(std::string name, const std::vector<variable::YoVariable*>& variables);

   const std::string& getName() const;

   bool isEmpty() const;
   std::size_t size() const;
   void clear();

   /** @throws std::out_of_range if index < 0 or index >= size(). */
   variable::YoVariable* get(int index) const;

   /**
    * Adds variable to this list, unless already present (nothing happens, returns false).
    *
    * @throws exceptions::NameCollisionException if a distinct instance of a variable with the same
    *         full name was previously added to this list.
    */
   bool add(variable::YoVariable* variable);

   /** @throws std::out_of_range if index < 0 or index > size(). */
   void add(int index, variable::YoVariable* variable);

   /**
    * Replaces the variable at index with variable, returning the variable previously there, or
    * nullptr if variable was already present elsewhere in the list (nothing happens in that case).
    *
    * @throws std::out_of_range if index < 0 or index >= size().
    */
   variable::YoVariable* set(int index, variable::YoVariable* variable);

   /** @throws std::out_of_range if index < 0 or index >= size(). */
   variable::YoVariable* remove(int index);

   /** @return true if variable was found and removed. */
   bool remove(variable::YoVariable* variable);

   void addAll(const std::vector<variable::YoVariable*>& variables);
   void addAll(const YoVariableList& other);

   /** The index of variable in this list, or -1 if not present. */
   int indexOf(const variable::YoVariable* variable) const;
   bool contains(const variable::YoVariable* variable) const;

   std::string toString() const;

   // YoVariableHolder:
   std::vector<variable::YoVariable*> getVariables() override;
   variable::YoVariable* findVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) override;
   std::vector<variable::YoVariable*> findVariables(const std::optional<std::string>& namespaceEnding, const std::string& name) override;
   std::vector<variable::YoVariable*> findVariables(const YoNamespace& namespaceValue) override;
   bool hasUniqueVariable(const std::optional<std::string>& namespaceEnding, const std::string& name) override;

   using YoVariableHolder::findVariable;
   using YoVariableHolder::findVariables;
   using YoVariableHolder::hasUniqueVariable;
   using YoVariableHolder::hasVariable;

private:
   bool registerVariableInMap(variable::YoVariable* variable);
   int countNumberOfVariables(const std::optional<std::string>& parentNamespace, const std::string& name) const;

   std::string name_;
   std::vector<variable::YoVariable*> variableList_;
   std::unordered_map<std::string, std::vector<variable::YoVariable*>> simpleNameToVariablesMap_;
};
}
