#include <gtest/gtest.h>

#include <vector>

#include "ihmc/yovariables/buffer/key_points_handler.h"

namespace ihmc::yovariables::buffer
{
namespace
{
class KeyPointsHandlerTest : public ::testing::Test
{
protected:
   void clearAndFillKeyPoints(const std::vector<int>& keyPointTimes)
   {
      keyPoints.clear();
      for (int keyPointTime : keyPointTimes)
         keyPoints.toggleKeyPoint(keyPointTime);
   }

   KeyPointsHandler keyPoints;
};

TEST_F(KeyPointsHandlerTest, testSetKeyPoint)
{
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 0u);
   EXPECT_TRUE(keyPoints.toggleKeyPoint(0));
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 1u);
   EXPECT_TRUE(keyPoints.toggleKeyPoint(4));
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 2u);
   EXPECT_TRUE(keyPoints.toggleKeyPoint(3));
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 3u);
}

TEST_F(KeyPointsHandlerTest, testRemoveDuplicateKeyPoint)
{
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 0u);
   EXPECT_TRUE(keyPoints.toggleKeyPoint(3));
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 1u);

   EXPECT_FALSE(keyPoints.toggleKeyPoint(3));
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 0u);
}

TEST_F(KeyPointsHandlerTest, testGetNextTime)
{
   std::vector<int> keyPointTimes{3, 16, 20, 48, 75};

   EXPECT_EQ(keyPoints.getNextKeyPoint(88), 88);

   clearAndFillKeyPoints(keyPointTimes);

   EXPECT_EQ(keyPoints.getNextKeyPoint(17), 20);
   EXPECT_EQ(keyPoints.getNextKeyPoint(99), 3);
}

TEST_F(KeyPointsHandlerTest, getPreviousTime)
{
   std::vector<int> keyPointTimes{3, 16, 20, 48, 75};

   EXPECT_EQ(keyPoints.getPreviousKeyPoint(1), 1);

   clearAndFillKeyPoints(keyPointTimes);

   EXPECT_EQ(keyPoints.getPreviousKeyPoint(47), 20);
   EXPECT_EQ(keyPoints.getPreviousKeyPoint(1), 75);
}

TEST_F(KeyPointsHandlerTest, testTrim)
{
   std::vector<int> keyPointTimes{3, 16, 20, 48, 75};

   clearAndFillKeyPoints(keyPointTimes);

   EXPECT_EQ(keyPoints.getKeyPoints().size(), keyPointTimes.size());

   keyPoints.trimKeyPoints(17, 47);
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 1u);
   EXPECT_EQ(keyPoints.getNextKeyPoint(1), 20);

   clearAndFillKeyPoints(keyPointTimes);

   keyPoints.trimKeyPoints(47, 17);
   EXPECT_EQ(keyPoints.getKeyPoints().size(), 4u);
   EXPECT_EQ(keyPoints.getKeyPoints()[0], 3);
   EXPECT_EQ(keyPoints.getKeyPoints()[1], 16);
   EXPECT_EQ(keyPoints.getKeyPoints()[2], 48);
   EXPECT_EQ(keyPoints.getKeyPoints()[3], 75);
}

TEST_F(KeyPointsHandlerTest, testUseKeyPoints)
{
   EXPECT_FALSE(keyPoints.areKeyPointsEnabled());
   keyPoints.enableKeyPoints(true);
   EXPECT_TRUE(keyPoints.areKeyPointsEnabled());
   keyPoints.enableKeyPoints(false);
   EXPECT_FALSE(keyPoints.areKeyPointsEnabled());
}
} // namespace
} // namespace ihmc::yovariables::buffer
