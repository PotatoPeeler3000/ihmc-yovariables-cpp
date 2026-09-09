#include <gtest/gtest.h>

#include <thread>
#include <vector>

#include "ihmc/yovariables/buffer/yo_buffer.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

using namespace ihmc::yovariables;

TEST(Phase2Smoke, TickWriteAndReadRoundTrip)
{
   registry::YoRegistry root("root");
   variable::YoDouble x("x", &root);

   buffer::YoBuffer yoBuffer(5);
   buffer::YoBufferVariableEntry& entry = yoBuffer.addVariable(x);

   for (int i = 0; i < 5; i++)
   {
      x.set(static_cast<double>(i) * 10.0);
      yoBuffer.tickAndWriteIntoBuffer();
   }

   EXPECT_EQ(yoBuffer.getCurrentIndex(), 0);

   // tickAndWriteIntoBuffer increments the index before writing, so index i+1 holds the value
   // written during the i-th iteration (with index 0 wrapping around to hold the last write).
   yoBuffer.setCurrentIndex(2);
   EXPECT_DOUBLE_EQ(x.getValue(), 10.0);

   EXPECT_DOUBLE_EQ(entry.readBufferAt(2), 10.0);
   EXPECT_DOUBLE_EQ(entry.readBufferAt(0), 40.0);

   buffer::YoBufferBounds bounds = entry.getBounds();
   EXPECT_DOUBLE_EQ(bounds.getLowerBound(), 0.0);
   EXPECT_DOUBLE_EQ(bounds.getUpperBound(), 40.0);
}

TEST(Phase2Smoke, FindVariableEntryByNameAndNamespace)
{
   registry::YoRegistry root("root");
   registry::YoRegistry child("child");
   root.addChild(&child);
   variable::YoDouble x("x", &child);

   buffer::YoBuffer yoBuffer(3);
   yoBuffer.addVariable(x);

   buffer::YoBufferVariableEntry* found = yoBuffer.findVariableEntry("x");
   ASSERT_NE(found, nullptr);
   EXPECT_EQ(&found->getVariable(), &x);

   EXPECT_EQ(yoBuffer.findVariableEntry("nope"), nullptr);
}

TEST(Phase2Smoke, KeyPointsOrderedAndTrimmed)
{
   buffer::KeyPointsHandler handler;
   handler.addKeyPoint(5);
   handler.addKeyPoint(1);
   handler.addKeyPoint(3);

   std::vector<int> expected = {1, 3, 5};
   EXPECT_EQ(handler.getKeyPoints(), expected);

   EXPECT_EQ(handler.getNextKeyPoint(2), 3);
   EXPECT_EQ(handler.getPreviousKeyPoint(4), 3);

   handler.trimKeyPoints(2, 6);
   std::vector<int> expectedAfterTrim = {3, 5};
   EXPECT_EQ(handler.getKeyPoints(), expectedAfterTrim);
}

TEST(Phase2Smoke, ConcurrentWriteAndReadIsSafe)
{
   registry::YoRegistry root("root");
   variable::YoDouble x("x", &root);
   buffer::YoBufferVariableEntry entry(x, 1000);

   std::atomic<bool> stop{false};
   std::thread writer([&]() {
      int i = 0;
      while (!stop.load())
      {
         entry.writeIntoBufferAt(i % 1000);
         i++;
      }
   });

   for (int i = 0; i < 2000; i++)
   {
      entry.getBounds();
      entry.readBufferAt(i % 1000);
   }

   stop.store(true);
   writer.join();

   SUCCEED();
}
