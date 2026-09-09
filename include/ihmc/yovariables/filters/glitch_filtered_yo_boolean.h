#pragma once

#include <optional>

#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_integer.h"

namespace ihmc::yovariables::filters
{
/**
 * Only accepts a new value once it has been observed for at least windowSize consecutive
 * update() calls, filtering out short "glitches".
 * <p>
 * Note: set(bool) here hides (does not override, since YoBoolean::set is not virtual)
 * YoBoolean::set(bool) - calling through a YoBoolean& reference to an instance of this class
 * bypasses the glitch counter reset that this override adds, unlike in the Java source where
 * virtual dispatch makes this transparent regardless of the static reference type.
 * </p>
 */
class GlitchFilteredYoBoolean : public variable::YoBoolean
{
public:
   /** @throws std::runtime_error if windowSize < 0. */
   GlitchFilteredYoBoolean(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                            variable::YoBoolean* yoVariableToFilter, variable::YoInteger& windowSize);
   GlitchFilteredYoBoolean(const std::string& name, const std::string& description, registry::YoRegistry* registry,
                            variable::YoBoolean* yoVariableToFilter, int windowSize);
   GlitchFilteredYoBoolean(const std::string& name, registry::YoRegistry* registry, int windowSize);

   bool set(bool value);

   /** @throws std::runtime_error if constructed without a variable to filter. */
   void update();
   void update(bool value);

   int getWindowSize() const;
   void setWindowSize(int windowSize);

private:
   variable::YoBoolean* variableToFilter_ = nullptr;
   variable::YoInteger* windowSize_ = nullptr;
   variable::YoInteger counter_;
   std::optional<variable::YoInteger> internalWindowSize_;
};
}
