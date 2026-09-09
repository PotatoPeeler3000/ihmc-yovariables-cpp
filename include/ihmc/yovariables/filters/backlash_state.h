#pragma once

namespace ihmc::yovariables::filters
{
enum class BacklashState
{
   BACKWARD_OK,
   FORWARD_OK,
   BACKWARD_SLOP,
   FORWARD_SLOP
};

inline bool isInBacklash(BacklashState state)
{
   return state == BacklashState::BACKWARD_SLOP || state == BacklashState::FORWARD_SLOP;
}
}
