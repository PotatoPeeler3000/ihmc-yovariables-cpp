#pragma once

namespace ihmc::yovariables::filters
{
class ProcessingYoVariable
{
public:
   virtual ~ProcessingYoVariable() = default;
   virtual void update() = 0;
   virtual void reset()
   {
   }
};
}
