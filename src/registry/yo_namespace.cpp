#include "ihmc/yovariables/registry/yo_namespace.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include "ihmc/yovariables/tools/yo_tools.h"

namespace ihmc::yovariables::registry
{
namespace
{
bool equalsIgnoreCase(const std::string& a, const std::string& b)
{
   if (a.size() != b.size())
      return false;
   return std::equal(a.begin(),
                      a.end(),
                      b.begin(),
                      [](unsigned char c1, unsigned char c2) { return std::tolower(c1) == std::tolower(c2); });
}
} // namespace

YoNamespace::YoNamespace(std::vector<std::string> subNames) : subNames_(std::move(subNames))
{
   if (subNames_.empty())
      throw std::invalid_argument("Cannot create an empty namespace.");

   name_ = tools::joinNames(subNames_);
}

YoNamespace::YoNamespace(const std::string& name) : name_(name)
{
   if (name.empty())
      throw std::invalid_argument("Cannot create an empty namespace.");

   subNames_ = tools::splitName(name);

   for (const std::string& subName : subNames_)
   {
      if (subName.empty())
         throw std::invalid_argument("Cannot create a namespace with empty sub-names: " + name);
   }
}

void YoNamespace::checkSanity() const
{
   tools::checkNamespaceSanity(*this);
}

const std::string& YoNamespace::getName() const
{
   return name_;
}

bool YoNamespace::isRoot() const
{
   return subNames_.size() == 1;
}

const std::string& YoNamespace::getRootName() const
{
   return subNames_.front();
}

const std::string& YoNamespace::getShortName() const
{
   return subNames_.back();
}

const std::vector<std::string>& YoNamespace::getSubNames() const
{
   return subNames_;
}

std::size_t YoNamespace::size() const
{
   return subNames_.size();
}

const std::string& YoNamespace::getSubName(std::size_t index) const
{
   return subNames_.at(index);
}

std::optional<YoNamespace> YoNamespace::getParent() const
{
   return removeEnd(1);
}

std::optional<YoNamespace> YoNamespace::subNamespace(std::size_t fromIndex) const
{
   return subNamespace(fromIndex, size());
}

std::optional<YoNamespace> YoNamespace::subNamespace(std::size_t fromIndex, std::size_t toIndex) const
{
   if (fromIndex == 0 && toIndex == size())
      return *this;
   if (fromIndex == toIndex)
      return std::nullopt;
   return YoNamespace(std::vector<std::string>(subNames_.begin() + fromIndex, subNames_.begin() + toIndex));
}

std::optional<YoNamespace> YoNamespace::removeStart(std::size_t length) const
{
   if (length > size())
      throw std::out_of_range("Invalid length: " + std::to_string(length));
   return subNamespace(length);
}

std::optional<YoNamespace> YoNamespace::removeStart(const YoNamespace& namespaceToRemove) const
{
   if (*this == namespaceToRemove || !startsWith(namespaceToRemove))
      return std::nullopt;
   return removeStart(namespaceToRemove.size());
}

std::optional<YoNamespace> YoNamespace::removeEnd(std::size_t length) const
{
   if (length > size())
      throw std::out_of_range("Invalid length: " + std::to_string(length));
   return subNamespace(0, size() - length);
}

std::optional<YoNamespace> YoNamespace::removeEnd(const YoNamespace& namespaceToRemove) const
{
   if (*this == namespaceToRemove || !endsWith(namespaceToRemove))
      return std::nullopt;
   return removeEnd(namespaceToRemove.size());
}

YoNamespace YoNamespace::prepend(const YoNamespace& other) const
{
   return tools::concatenate(other, *this);
}

YoNamespace YoNamespace::prepend(const std::string& name) const
{
   return tools::concatenate(name, *this);
}

YoNamespace YoNamespace::append(const YoNamespace& other) const
{
   return tools::concatenate(*this, other);
}

YoNamespace YoNamespace::append(const std::string& name) const
{
   return tools::concatenate(*this, name);
}

bool YoNamespace::endsWith(const YoNamespace& query, bool ignoreCase) const
{
   if (query.size() > size())
      return false;

   for (std::size_t i = 1; i <= query.size(); i++)
   {
      const std::string& querySubName = query.subNames_[query.size() - i];
      const std::string& thisSubName = subNames_[size() - i];

      bool equal = ignoreCase ? equalsIgnoreCase(querySubName, thisSubName) : querySubName == thisSubName;
      if (!equal)
         return false;
   }

   return true;
}

bool YoNamespace::endsWith(const std::string& nameToMatch, bool ignoreCase) const
{
   if (nameToMatch.size() > name_.size())
      return false;

   bool matches = ignoreCase ? equalsIgnoreCase(name_.substr(name_.size() - nameToMatch.size()), nameToMatch)
                              : name_.size() >= nameToMatch.size() && name_.compare(name_.size() - nameToMatch.size(), nameToMatch.size(), nameToMatch) == 0;
   if (!matches)
      return false;

   if (name_.size() == nameToMatch.size())
      return true;

   return name_[name_.size() - nameToMatch.size() - 1] == tools::kNamespaceSeparator;
}

bool YoNamespace::startsWith(const YoNamespace& query, bool ignoreCase) const
{
   if (query.size() > size())
      return false;

   for (std::size_t i = 0; i < query.size(); i++)
   {
      bool equal = ignoreCase ? equalsIgnoreCase(query.subNames_[i], subNames_[i]) : query.subNames_[i] == subNames_[i];
      if (!equal)
         return false;
   }

   return true;
}

bool YoNamespace::startsWith(const std::string& nameToMatch, bool ignoreCase) const
{
   if (nameToMatch.size() > name_.size())
      return false;

   bool matches = ignoreCase ? equalsIgnoreCase(name_.substr(0, nameToMatch.size()), nameToMatch) : name_.compare(0, nameToMatch.size(), nameToMatch) == 0;
   if (!matches)
      return false;

   if (name_.size() == nameToMatch.size())
      return true;

   return name_[nameToMatch.size()] == tools::kNamespaceSeparator;
}

bool YoNamespace::contains(const YoNamespace& query, bool ignoreCase) const
{
   if (query.size() > size())
      return false;

   std::size_t startIndex = static_cast<std::size_t>(-1);
   const std::string& queryFirstSubName = query.subNames_.front();

   for (std::size_t i = 0; i + query.size() <= subNames_.size(); i++)
   {
      bool equal = ignoreCase ? equalsIgnoreCase(subNames_[i], queryFirstSubName) : subNames_[i] == queryFirstSubName;
      if (equal)
      {
         startIndex = i;
         break;
      }
   }

   if (startIndex == static_cast<std::size_t>(-1))
      return false;

   for (std::size_t i = 1; i < query.size(); i++)
   {
      bool equal = ignoreCase ? equalsIgnoreCase(query.subNames_[i], subNames_[startIndex + i]) : query.subNames_[i] == subNames_[startIndex + i];
      if (!equal)
         return false;
   }

   return true;
}

bool YoNamespace::contains(const std::string& nameToMatch, bool ignoreCase) const
{
   std::size_t startIndex;
   if (ignoreCase)
   {
      auto it = std::search(name_.begin(),
                             name_.end(),
                             nameToMatch.begin(),
                             nameToMatch.end(),
                             [](unsigned char c1, unsigned char c2) { return std::tolower(c1) == std::tolower(c2); });
      startIndex = it == name_.end() ? std::string::npos : static_cast<std::size_t>(it - name_.begin());
   }
   else
   {
      startIndex = name_.find(nameToMatch);
   }

   if (startIndex == std::string::npos)
      return false;

   std::size_t endIndex = startIndex + nameToMatch.size();

   if (startIndex > 0 && name_[startIndex - 1] != tools::kNamespaceSeparator)
      return false;

   if (endIndex < name_.size() && name_[endIndex] != tools::kNamespaceSeparator)
      return false;

   return true;
}

bool YoNamespace::operator==(const YoNamespace& other) const
{
   return name_ == other.name_;
}

bool YoNamespace::operator!=(const YoNamespace& other) const
{
   return !(*this == other);
}

std::string YoNamespace::toString() const
{
   return name_;
}
}
