#include "ihmc/yovariables/tools/yo_geometry_name_tools.h"

#include <algorithm>

namespace ihmc::yovariables::tools
{
namespace
{
std::string reversed(const std::string& value)
{
   return std::string(value.rbegin(), value.rend());
}

std::string uncapitalize(const std::string& value)
{
   if (value.empty())
      return value;
   std::string result = value;
   result[0] = static_cast<char>(std::tolower(static_cast<unsigned char>(result[0])));
   return result;
}

std::string capitalize(const std::string& value)
{
   if (value.empty())
      return value;
   std::string result = value;
   result[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(result[0])));
   return result;
}
} // namespace

std::string getCommonPrefix(const std::vector<std::string>& strings)
{
   if (strings.empty())
      return "";

   std::string prefix = strings.front();
   for (const std::string& value : strings)
   {
      std::size_t matchLength = 0;
      while (matchLength < prefix.size() && matchLength < value.size() && prefix[matchLength] == value[matchLength])
         matchLength++;
      prefix.resize(matchLength);
      if (prefix.empty())
         break;
   }
   return prefix;
}

std::string getCommonSuffix(const std::vector<std::string>& strings)
{
   if (strings.empty())
      return "";

   std::vector<std::string> reversedStrings;
   reversedStrings.reserve(strings.size());
   for (const std::string& value : strings)
      reversedStrings.push_back(reversed(value));

   return reversed(getCommonPrefix(reversedStrings));
}

std::string createXName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return assembleName({namePrefix, "x", nameSuffix});
}

std::string createYName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return assembleName({namePrefix, "y", nameSuffix});
}

std::string createZName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return assembleName({namePrefix, "z", nameSuffix});
}

std::string createQxName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return assembleName({namePrefix, "qx", nameSuffix});
}

std::string createQyName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return assembleName({namePrefix, "qy", nameSuffix});
}

std::string createQzName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return assembleName({namePrefix, "qz", nameSuffix});
}

std::string createQsName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return assembleName({namePrefix, "qs", nameSuffix});
}

std::string assembleName(const std::vector<std::string>& subNames)
{
   if (subNames.empty())
      return "";

   std::string name = subNames.front();
   for (std::size_t i = 1; i < subNames.size(); i++)
      name = appendSuffix(name, subNames[i]);
   return name;
}

std::string appendSuffix(const std::string& name, const std::string& suffix)
{
   if (name.empty())
      return uncapitalize(suffix);
   if (suffix.empty())
      return name;

   if (!name.empty() && name.back() == '_')
      return name + uncapitalize(suffix);
   return name + capitalize(suffix);
}
}
