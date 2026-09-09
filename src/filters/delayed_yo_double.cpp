#include "ihmc/yovariables/filters/delayed_yo_double.h"

namespace ihmc::yovariables::filters
{
DelayedYoDouble::DelayedYoDouble(const std::string& name, const std::string& description, providers::DoubleProvider& variableToDelay, int ticksToDelay,
                                  registry::YoRegistry* registry)
   : YoDouble(name, description, registry), variableToDelay_(variableToDelay)
{
   previousYoDouble_.reserve(static_cast<std::size_t>(ticksToDelay));
   for (int i = 0; i < ticksToDelay; i++)
   {
      auto previous = std::make_unique<variable::YoDouble>(name + "_previous" + std::to_string(i), registry);
      previous->set(variableToDelay_.getValue());
      previousYoDouble_.push_back(std::move(previous));
   }

   set(variableToDelay_.getValue());
}

void DelayedYoDouble::update()
{
   if (previousYoDouble_.empty())
   {
      set(variableToDelay_.getValue());
      return;
   }

   set(previousYoDouble_.front()->getValue());

   for (std::size_t i = 0; i + 1 < previousYoDouble_.size(); i++)
      previousYoDouble_[i]->set(previousYoDouble_[i + 1]->getValue());

   previousYoDouble_.back()->set(variableToDelay_.getValue());
}

void DelayedYoDouble::reset()
{
   for (std::unique_ptr<variable::YoDouble>& previous : previousYoDouble_)
      previous->set(variableToDelay_.getValue());
   set(variableToDelay_.getValue());
}
}
