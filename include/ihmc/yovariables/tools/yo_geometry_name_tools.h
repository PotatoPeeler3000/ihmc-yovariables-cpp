#pragma once

#include <string>
#include <vector>

namespace ihmc::yovariables::tools
{
/**
 * Tools that help standardize name creation for classes implementing Euclid types with
 * YoVariables (e.g. a hypothetical YoPoint2D/YoFrameQuaternion) - kept here even though the
 * `euclid` package itself is out of scope for this port, since nothing in these functions actually
 * depends on Euclid types (the Java source's imports of them are for javadoc @link references only).
 */

/** The largest prefix common to all of strings, or "" if none/empty. */
std::string getCommonPrefix(const std::vector<std::string>& strings);

/** The largest suffix common to all of strings, or "" if none/empty. */
std::string getCommonSuffix(const std::vector<std::string>& strings);

std::string createXName(const std::string& namePrefix, const std::string& nameSuffix);
std::string createYName(const std::string& namePrefix, const std::string& nameSuffix);
std::string createZName(const std::string& namePrefix, const std::string& nameSuffix);
std::string createQxName(const std::string& namePrefix, const std::string& nameSuffix);
std::string createQyName(const std::string& namePrefix, const std::string& nameSuffix);
std::string createQzName(const std::string& namePrefix, const std::string& nameSuffix);
std::string createQsName(const std::string& namePrefix, const std::string& nameSuffix);

/** Concatenates subNames into a single name, managing the casing of each appended sub-name. */
std::string assembleName(const std::vector<std::string>& subNames);

/**
 * Appends suffix to name, lower-casing suffix's first character if name is empty or ends with
 * '_', upper-casing it otherwise.
 */
std::string appendSuffix(const std::string& name, const std::string& suffix);
}
