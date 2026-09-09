#pragma once

namespace ihmc::yovariables::variable
{
/** Enumerates the primitive kinds implemented as YoVariable (and YoParameter). */
enum class YoVariableType
{
   DOUBLE,
   BOOLEAN,
   ENUM,
   INTEGER,
   LONG
};
}
