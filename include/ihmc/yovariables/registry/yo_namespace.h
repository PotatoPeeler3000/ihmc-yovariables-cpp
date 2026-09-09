#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace ihmc::yovariables::registry
{
/**
 * A namespace composed of sub-names, where typically each sub-name is the name of a YoRegistry
 * with a parent-to-child relationship between one sub-name and the following one.
 */
class YoNamespace
{
public:
   /** Creates a namespace from an ordered list of names starting with the name of the root. */
   explicit YoNamespace(std::vector<std::string> subNames);

   /**
    * Creates a namespace from its full string representation, i.e. sub-names joined by the
    * namespace separator.
    */
   explicit YoNamespace(const std::string& name);

   /** Performs sanity checks (no empty/duplicate sub-names, no illegal separator use). */
   void checkSanity() const;

   const std::string& getName() const;

   /** True if this namespace is a single sub-name (i.e. it references the root element). */
   bool isRoot() const;

   const std::string& getRootName() const;

   /** The last sub-name of this namespace. */
   const std::string& getShortName() const;

   const std::vector<std::string>& getSubNames() const;

   std::size_t size() const;

   const std::string& getSubName(std::size_t index) const;

   /**
    * A new namespace equal to this one minus its last sub-name, or std::nullopt if this namespace
    * is already a root (single sub-name) namespace.
    */
   std::optional<YoNamespace> getParent() const;

   /**
    * A new namespace containing the sub-names in [fromIndex, toIndex), or std::nullopt if
    * fromIndex == toIndex.
    */
   std::optional<YoNamespace> subNamespace(std::size_t fromIndex) const;
   std::optional<YoNamespace> subNamespace(std::size_t fromIndex, std::size_t toIndex) const;

   /**
    * A new namespace that is this one minus length sub-names from the start, or std::nullopt if
    * length == size().
    */
   std::optional<YoNamespace> removeStart(std::size_t length) const;

   /**
    * A new namespace that is this one with namespaceToRemove removed from the start, or
    * std::nullopt if namespaceToRemove does not prefix this namespace or equals it.
    */
   std::optional<YoNamespace> removeStart(const YoNamespace& namespaceToRemove) const;

   /**
    * A new namespace that is this one minus length sub-names from the end, or std::nullopt if
    * length == size().
    */
   std::optional<YoNamespace> removeEnd(std::size_t length) const;

   std::optional<YoNamespace> removeEnd(const YoNamespace& namespaceToRemove) const;

   /** A new namespace: [other, this]. */
   YoNamespace prepend(const YoNamespace& other) const;
   YoNamespace prepend(const std::string& name) const;

   /** A new namespace: [this, other]. */
   YoNamespace append(const YoNamespace& other) const;
   YoNamespace append(const std::string& name) const;

   bool endsWith(const YoNamespace& query, bool ignoreCase = false) const;
   bool endsWith(const std::string& nameToMatch, bool ignoreCase = false) const;

   bool startsWith(const YoNamespace& query, bool ignoreCase = false) const;
   bool startsWith(const std::string& nameToMatch, bool ignoreCase = false) const;

   bool contains(const YoNamespace& query, bool ignoreCase = false) const;
   bool contains(const std::string& nameToMatch, bool ignoreCase = false) const;

   bool operator==(const YoNamespace& other) const;
   bool operator!=(const YoNamespace& other) const;

   std::string toString() const;

private:
   std::string name_;
   std::vector<std::string> subNames_;
};
}

namespace std
{
template <>
struct hash<ihmc::yovariables::registry::YoNamespace>
{
   std::size_t operator()(const ihmc::yovariables::registry::YoNamespace& namespaceValue) const noexcept
   {
      return std::hash<std::string>{}(namespaceValue.getName());
   }
};
}
