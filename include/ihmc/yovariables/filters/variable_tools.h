#pragma once

#include <string>

namespace ihmc::yovariables::filters
{
/**
 * Naming-convention helpers for the auxiliary YoVariables a filter class registers alongside
 * itself.
 * <p>
 * Java's VariableTools directly constructs and returns each auxiliary YoVariable/YoBoolean/
 * YoInteger. That doesn't translate directly to C++: YoVariable subclasses are neither copyable
 * nor movable (matching their non-owning-pointer, construct-in-place design from earlier phases),
 * so a factory function cannot construct one and hand it back by value. Instead, each filter class
 * constructs its own auxiliary variables directly as members, using these functions only for the
 * naming convention Java's VariableTools centralized.
 * </p>
 */
inline std::string hasBeenCalledName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return namePrefix + "HasBeenCalled" + nameSuffix;
}

inline std::string limitedCalledName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return namePrefix + "Limited" + nameSuffix;
}

inline std::string maxRateName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return namePrefix + "MaxRate" + nameSuffix;
}

inline std::string alphaVariableName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return namePrefix + "AlphaVariable" + nameSuffix;
}

inline std::string windowSizeName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return namePrefix + "WindowSize" + nameSuffix;
}

inline std::string maxAccelerationName(const std::string& namePrefix, const std::string& nameSuffix)
{
   return namePrefix + "MaxAcceleration" + nameSuffix;
}
}
