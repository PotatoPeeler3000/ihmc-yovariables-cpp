#pragma once

namespace ihmc::yovariables::filters
{
/**
 * Free-standing rather than nested inside SecondOrderFilteredYoDouble (as in the Java source, as
 * SecondOrderFilteredYoDouble.SecondOrderFilterType): SecondOrderFilteredYoVariableParameters
 * needs to reference it before SecondOrderFilteredYoDouble is defined, and C++ cannot forward-
 * declare a class's nested enum ahead of the class itself.
 */
enum class SecondOrderFilterType
{
   LOW_PASS,
   NOTCH,
   BAND,
   HIGH_PASS
};
}
