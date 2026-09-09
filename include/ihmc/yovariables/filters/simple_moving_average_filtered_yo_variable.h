#pragma once

#include <vector>

#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_integer.h"

namespace ihmc::yovariables::filters
{
/**
 * Filters a YoVariable using a moving average filter. NOT rewindable.
 * <p>
 * The Java source backs its circular buffer with an EJML DMatrixRMaj, used only as a resizable
 * single-column array (no matrix operations) - ported here as a plain std::vector<double> instead,
 * since that is a faithful, EJML-free equivalent of how it's actually used.
 * </p>
 */
class SimpleMovingAverageFilteredYoVariable : public variable::YoDouble
{
public:
   SimpleMovingAverageFilteredYoVariable(const std::string& name, int windowSize, registry::YoRegistry* registry);
   SimpleMovingAverageFilteredYoVariable(const std::string& name, int windowSize, variable::YoDouble* yoVariableToFilter, registry::YoRegistry* registry);

   void setWindowSize(int windowSize);

   void update();
   void update(double value);
   void reset();

   bool getHasBufferWindowFilled() const;

private:
   variable::YoInteger windowSize_;
   variable::YoDouble* yoVariableToFilter_ = nullptr;

   std::vector<double> previousUpdateValues_;
   int bufferPosition_ = 0;
   bool bufferHasBeenFilled_ = false;
};
}
