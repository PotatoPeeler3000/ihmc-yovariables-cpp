#include <gtest/gtest.h>

#include "ihmc/yovariables/version.h"

TEST(Version, IsNonEmpty)
{
   EXPECT_GT(std::string(ihmc::yovariables::version).size(), 0u);
}
