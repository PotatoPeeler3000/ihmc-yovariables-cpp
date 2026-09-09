#pragma once

namespace ihmc::yovariables::parameters
{
/** States relating to initialization of a YoParameter. */
enum class ParameterLoadStatus
{
   /** Not yet initialized; cannot be used. */
   UNLOADED,
   /** Initialized using its default value given at construction. */
   DEFAULT,
   /** Initialized using an external source such as an XML file. */
   LOADED
};
}
