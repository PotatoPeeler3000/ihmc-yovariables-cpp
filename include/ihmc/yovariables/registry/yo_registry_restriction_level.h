#pragma once

namespace ihmc::yovariables::registry
{
/**
 * Declared in order of restriction: when comparing two levels, the one with the greater
 * underlying value is the more restrictive.
 */
enum class YoRegistryRestrictionLevel
{
   /** Structure of this registry and its descendants can be freely added to and removed from. */
   FULLY_MUTABLE = 0,
   /** Structure can only be expanded: additions allowed, removals are not. */
   RESTRICTED = 1,
   /** Structure is frozen: neither additions nor removals are allowed. */
   IMMUTABLE = 2
};

inline bool isAdditionAllowed(YoRegistryRestrictionLevel level)
{
   return level != YoRegistryRestrictionLevel::IMMUTABLE;
}

inline bool isRemovalAllowed(YoRegistryRestrictionLevel level)
{
   return level == YoRegistryRestrictionLevel::FULLY_MUTABLE;
}
}
