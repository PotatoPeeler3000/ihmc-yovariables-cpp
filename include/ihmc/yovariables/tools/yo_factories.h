#pragma once

#include <memory>
#include <vector>

namespace ihmc::yovariables::registry
{
class YoNamespace;
class YoRegistry;
}

namespace ihmc::yovariables::tools
{
/** Factories for YoRegistry. */

/**
 * Finds a registry with namespace fullNamespace in startRegistry's subtree, creating and
 * attaching one (and any missing ancestors) if none exists.
 * <p>
 * Any newly created registries are appended, in root-to-leaf order, to
 * newRegistriesOwnershipSink, which the caller must keep alive for as long as the returned
 * registry needs to remain part of the tree - unlike Java, where new registries stay alive simply
 * by being reachable from startRegistry via garbage collection, C++ needs an explicit owner. When
 * the result is a pre-existing registry (already part of startRegistry's subtree, including
 * startRegistry itself), nothing is appended.
 * </p>
 *
 * @return the found/created registry, or nullptr if fullNamespace is incompatible with startRegistry.
 */
registry::YoRegistry* findOrCreateRegistry(registry::YoRegistry& startRegistry, const registry::YoNamespace& fullNamespace,
                                            std::vector<std::unique_ptr<registry::YoRegistry>>& newRegistriesOwnershipSink);

/**
 * Creates a chain of registries such that the last (childless) one's namespace equals
 * fullNamespace, each one the previous one's only child.
 *
 * @return the chain, root first and leaf last. The caller owns the whole chain (see
 *         findOrCreateRegistry's ownership note).
 */
std::vector<std::unique_ptr<registry::YoRegistry>> createChainOfRegistries(const registry::YoNamespace& fullNamespace);
}
