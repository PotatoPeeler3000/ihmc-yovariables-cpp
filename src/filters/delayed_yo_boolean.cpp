#include "ihmc/yovariables/filters/delayed_yo_boolean.h"

namespace ihmc::yovariables::filters
{
DelayedYoBoolean::DelayedYoBoolean(const std::string& name, const std::string& description, variable::YoBoolean& variableToDelay, int ticksToDelay,
                                    registry::YoRegistry* registry)
   : YoBoolean(name, description, registry), variableToDelay_(variableToDelay)
{
   previousYoVariables_.reserve(static_cast<std::size_t>(ticksToDelay));
   for (int i = 0; i < ticksToDelay; i++)
   {
      auto previous = std::make_unique<variable::YoBoolean>(name + "_previous" + std::to_string(i), registry);
      previous->set(variableToDelay_.getBooleanValue());
      previousYoVariables_.push_back(std::move(previous));
   }

   set(variableToDelay_.getBooleanValue());
}

void DelayedYoBoolean::update()
{
   if (previousYoVariables_.empty())
   {
      set(variableToDelay_.getBooleanValue());
      return;
   }

   set(previousYoVariables_.front()->getBooleanValue());

   for (std::size_t i = 0; i + 1 < previousYoVariables_.size(); i++)
      previousYoVariables_[i]->set(previousYoVariables_[i + 1]->getBooleanValue());

   previousYoVariables_.back()->set(variableToDelay_.getBooleanValue());
}

void DelayedYoBoolean::reset()
{
   for (std::unique_ptr<variable::YoBoolean>& previous : previousYoVariables_)
      previous->set(variableToDelay_.getBooleanValue());
   set(variableToDelay_.getBooleanValue());
}
}
