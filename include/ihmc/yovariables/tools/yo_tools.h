#pragma once

#include <functional>
#include <iosfwd>
#include <string>
#include <string_view>
#include <vector>

namespace ihmc::yovariables::registry
{
class YoNamespace;
class YoRegistry;
}

namespace ihmc::yovariables::tools
{
/** General tools used by YoNamespace, YoRegistry, and YoVariable. */

/** Character used to separate sub-names in a namespace's string representation. */
inline constexpr char kNamespaceSeparator = '.';
inline constexpr std::string_view kNamespaceSeparatorString = ".";

/**
 * Characters that cannot appear in the name of a YoRegistry or YoVariable: any punctuation
 * character except underscore and hyphen, plus space.
 */
inline constexpr std::string_view kIllegalCharacters = " `~.*!?@#$%/^&()<>,:;{}'\"\\=+|";

/**
 * Checks that the given name does not contain any illegal character.
 *
 * @throws exceptions::IllegalNameException if the name contains at least one illegal character.
 */
void checkForIllegalCharacters(const std::string& name);

/** Checks that the given name does not contain the namespace separator. */
void checkNameDoesNotContainSeparator(const std::string& name);

/**
 * Splits name at every occurrence of the namespace separator. Unlike a typical string split,
 * this preserves empty sub-names (including a trailing one), matching Java's
 * {@code String.split(regex, -1)} used by the original implementation.
 */
std::vector<std::string> splitName(const std::string& name);

/** Joins subNames into a single string, separated by the namespace separator. */
std::string joinNames(const std::vector<std::string>& subNames);

/**
 * Validates a namespace: no empty sub-name, no sub-name containing the separator, and no
 * duplicate sub-names.
 *
 * @throws exceptions::IllegalNameException if any check fails.
 */
void checkNamespaceSanity(const registry::YoNamespace& namespaceToCheck);

/** Concatenates two namespaces: [namespaceA, namespaceB]. */
registry::YoNamespace concatenate(const registry::YoNamespace& namespaceA, const registry::YoNamespace& namespaceB);

/** Appends name (which may itself represent a namespace) to namespaceValue. */
registry::YoNamespace concatenate(const registry::YoNamespace& namespaceValue, const std::string& name);

/** Prepends name (which may itself represent a namespace) to namespaceValue. */
registry::YoNamespace concatenate(const std::string& name, const registry::YoNamespace& namespaceValue);

/** Creates a new namespace that starts with nameA and ends with nameB. */
registry::YoNamespace concatenate(const std::string& nameA, const std::string& nameB);

/**
 * Isolates the substring after the last namespace separator - the last sub-name if name
 * represents a namespace, or name unchanged if it contains no separator.
 */
std::string toShortName(const std::string& name);

/** registry's number of variables and children, formatted for aligned printing alongside others. */
std::string getRegistryInfo(const registry::YoRegistry& registry, int maxNameLength = 70);

/**
 * Recurses the subtree starting at root and prints, to printStream, the number of variables and
 * children for each registry matching filter, sorted by number of variables (descending).
 */
void printStatistics(const std::function<bool(const registry::YoRegistry&)>& filter, registry::YoRegistry& root,
                      const std::function<std::string(const registry::YoRegistry&)>& registryInfoFunction, std::ostream& printStream);

/**
 * Recurses the subtree starting at root and prints, to printStream, the number of variables and
 * children for every registry with at least minVariablesToPrint variables or minChildrenToPrint
 * children, sorted by number of variables (descending).
 */
void printStatistics(int minVariablesToPrint, int minChildrenToPrint, registry::YoRegistry& root,
                      const std::function<std::string(const registry::YoRegistry&)>& registryInfoFunction, std::ostream& printStream);

/** Same as the 5-argument overload, printing to std::cout via getRegistryInfo(registry). */
void printStatistics(int minVariablesToPrint, int minChildrenToPrint, registry::YoRegistry& root);
}
