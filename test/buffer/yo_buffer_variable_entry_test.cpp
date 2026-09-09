#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "ihmc/yovariables/buffer/yo_buffer_bounds.h"
#include "ihmc/yovariables/buffer/yo_buffer_variable_entry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::buffer
{
namespace
{
class YoBufferVariableEntryTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      yoDouble = std::make_unique<variable::YoDouble>("yoDouble", nullptr);
      yoDouble->set(0);
      dataBufferEntry = std::make_unique<YoBufferVariableEntry>(*yoDouble, nPoints);
   }

   const int nPoints = 10000;
   std::unique_ptr<variable::YoDouble> yoDouble;
   std::unique_ptr<YoBufferVariableEntry> dataBufferEntry;
};

TEST_F(YoBufferVariableEntryTest, testTickAndUpdate)
{
   std::mt19937 random(1345143U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   std::vector<double> tempData(nPoints);
   for (int i = 0; i < nPoints; i++)
   {
      tempData[i] = dist(random);
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   std::vector<double> data = dataBufferEntry->getBuffer();

   for (int i = 0; i < nPoints; i++)
      EXPECT_EQ(tempData[i], data[i]);
}

TEST_F(YoBufferVariableEntryTest, testComputeAverage)
{
   std::mt19937 random(768439U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   double lowerBound = -100.0;
   double upperBound = 100.0;

   double total = 0.0;
   for (int i = 0; i < nPoints; i++)
   {
      double data = dist(random) * (upperBound - lowerBound) + lowerBound;
      yoDouble->set(data);
      total += data;
      dataBufferEntry->writeIntoBufferAt(i);
   }

   double average = total / nPoints;
   double computedAverage = dataBufferEntry->computeAverage();

   EXPECT_NEAR(average, computedAverage, 1e-7);
}

TEST_F(YoBufferVariableEntryTest, testUpdateValue)
{
   std::mt19937 random(754380U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   std::vector<double> tempData(nPoints);
   for (int i = 0; i < nPoints; i++)
   {
      tempData[i] = dist(random);
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   for (int i = 0; i < nPoints; i++)
   {
      dataBufferEntry->readFromBufferAt(i);
      EXPECT_EQ(yoDouble->getValueAsDouble(), dataBufferEntry->getBuffer()[i]);
   }
}

TEST_F(YoBufferVariableEntryTest, testCheckIfDataIsEqual)
{
   std::mt19937 random(32890U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   variable::YoDouble yoDouble2("yoDouble", nullptr);
   yoDouble2.set(0);
   YoBufferVariableEntry entry2(yoDouble2, nPoints);

   for (int i = 0; i < nPoints; i++)
   {
      double temp = dist(random);

      yoDouble->set(temp);
      yoDouble2.set(temp);

      dataBufferEntry->writeIntoBufferAt(i);
      entry2.writeIntoBufferAt(i);
   }

   EXPECT_TRUE(dataBufferEntry->epsilonEquals(entry2, 0.0));

   dataBufferEntry = std::make_unique<YoBufferVariableEntry>(*yoDouble, nPoints);
   YoBufferVariableEntry entry2b(yoDouble2, nPoints);

   for (int i = 0; i < nPoints; i++)
   {
      double temp = dist(random);

      yoDouble->set(temp);
      yoDouble2.set(-temp);

      dataBufferEntry->writeIntoBufferAt(i);
      entry2b.writeIntoBufferAt(i);
   }

   EXPECT_FALSE(dataBufferEntry->epsilonEquals(entry2b, 0.0));
}

TEST_F(YoBufferVariableEntryTest, testGetMinAndMaxScaling)
{
   std::mt19937 random(80423U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   double minScaling = dist(random);
   double maxScaling = dist(random) + 2;
   dataBufferEntry->setCustomBounds(minScaling, maxScaling);

   EXPECT_EQ(minScaling, dataBufferEntry->getCustomLowerBound());
   EXPECT_EQ(maxScaling, dataBufferEntry->getCustomUpperBound());
}

TEST_F(YoBufferVariableEntryTest, testGetVariable)
{
   EXPECT_EQ(yoDouble.get(), &dataBufferEntry->getVariable());
}

TEST_F(YoBufferVariableEntryTest, testCopyValueThrough)
{
   std::mt19937 random(2346180U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   double tempDouble = dist(random);
   yoDouble->set(tempDouble);
   dataBufferEntry->writeIntoBufferAt(0);
   dataBufferEntry->fillBuffer();

   for (int i = 0; i < nPoints; i++)
      EXPECT_EQ(tempDouble, dataBufferEntry->getBuffer()[i]);
}

TEST_F(YoBufferVariableEntryTest, testEnlargeBufferSize)
{
   std::mt19937 random(324270U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);
   std::uniform_int_distribution<int> smallDist(0, 99);

   std::vector<double> tempData(nPoints);
   int newSizeDelta = smallDist(random);

   for (int i = 0; i < nPoints; i++)
   {
      tempData[i] = dist(random);
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }
   dataBufferEntry->enlargeBufferSize(nPoints + newSizeDelta);

   EXPECT_EQ(nPoints + newSizeDelta, static_cast<int>(dataBufferEntry->getBuffer().size()));

   for (int i = 0; i < nPoints; i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);
}

TEST_F(YoBufferVariableEntryTest, testCropData)
{
   std::mt19937 random(2372891U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   std::vector<double> tempData(nPoints);

   for (int i = 0; i < nPoints; i++)
   {
      tempData[i] = dist(random);
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   // Test Failure Conditions; data remains unchanged so only examine lengths
   EXPECT_EQ(-1, dataBufferEntry->cropBuffer(-1, nPoints));
   EXPECT_EQ(nPoints, static_cast<int>(dataBufferEntry->getBuffer().size()));
   EXPECT_EQ(-1, dataBufferEntry->cropBuffer(0, nPoints + 1));
   EXPECT_EQ(nPoints, static_cast<int>(dataBufferEntry->getBuffer().size()));

   // Test unchanged size
   EXPECT_EQ(nPoints, dataBufferEntry->cropBuffer(0, nPoints - 1));
   EXPECT_EQ(nPoints, static_cast<int>(dataBufferEntry->getBuffer().size()));

   // Verify data integrity
   for (int i = 0; i < nPoints; i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);

   // Test cropping from end
   EXPECT_EQ(nPoints - 100, dataBufferEntry->cropBuffer(0, nPoints - 101));

   // Verify data integrity
   for (int i = 0; i < nPoints - 100; i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);

   // Restore dataBufferEntry to original state.
   dataBufferEntry->enlargeBufferSize(nPoints);
   for (int i = 0; i < nPoints; i++)
   {
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   // Test cropping from beginning
   EXPECT_EQ(nPoints - 100, dataBufferEntry->cropBuffer(100, nPoints - 1));

   // Verify data integrity
   for (int i = 0; i < static_cast<int>(dataBufferEntry->getBuffer().size()); i++)
      EXPECT_EQ(tempData[100 + i], dataBufferEntry->getBuffer()[i]);
}

TEST_F(YoBufferVariableEntryTest, testCutData)
{
   std::mt19937 random(1230972U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   std::vector<double> tempData(nPoints);

   for (int i = 0; i < nPoints; i++)
   {
      tempData[i] = dist(random);
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   // Test Failure Conditions; data remains unchanged so only examine lengths
   EXPECT_EQ(-1, dataBufferEntry->cropBuffer(-1, nPoints));
   EXPECT_EQ(nPoints, static_cast<int>(dataBufferEntry->getBuffer().size()));
   EXPECT_EQ(-1, dataBufferEntry->cropBuffer(0, nPoints + 1));
   EXPECT_EQ(nPoints, static_cast<int>(dataBufferEntry->getBuffer().size()));

   // Test unchanged size
   EXPECT_EQ(-1, dataBufferEntry->cutBuffer(nPoints / 2 + 1, nPoints / 2 - 1));
   EXPECT_EQ(nPoints, static_cast<int>(dataBufferEntry->getBuffer().size()));

   // Verify data integrity
   for (int i = 0; i < nPoints; i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);

   // Test cut one point in the middle:
   int cutPoint = nPoints / 2;
   int sizeAfterCut = dataBufferEntry->cutBuffer(cutPoint, cutPoint);
   EXPECT_EQ(nPoints - 1, sizeAfterCut);
   EXPECT_EQ(nPoints - 1, static_cast<int>(dataBufferEntry->getBuffer().size()));

   // Verify data integrity
   for (int i = 0; i < cutPoint; i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);

   for (int i = cutPoint; i < nPoints - 1; i++)
      EXPECT_EQ(tempData[i + 1], dataBufferEntry->getBuffer()[i]);

   // Restore dataBufferEntry to original state.
   dataBufferEntry->enlargeBufferSize(nPoints);
   for (int i = 0; i < nPoints; i++)
   {
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   // Test cutting at beginning
   sizeAfterCut = dataBufferEntry->cutBuffer(0, 2);
   EXPECT_EQ(nPoints - 3, sizeAfterCut);

   // Verify data integrity
   for (int i = 0; i < nPoints - 3; i++)
      EXPECT_EQ(tempData[i + 3], dataBufferEntry->getBuffer()[i]);

   // Restore dataBufferEntry to original state.
   dataBufferEntry->enlargeBufferSize(nPoints);
   for (int i = 0; i < nPoints; i++)
   {
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   // Test cutting at end
   sizeAfterCut = dataBufferEntry->cutBuffer(nPoints - 3, nPoints - 1);
   EXPECT_EQ(nPoints - 3, sizeAfterCut);

   // Verify data integrity
   for (int i = 0; i < static_cast<int>(dataBufferEntry->getBuffer().size()); i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);
}

TEST_F(YoBufferVariableEntryTest, testPackData)
{
   std::mt19937 random(4357684U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);
   std::uniform_int_distribution<int> startDist(0, nPoints - 2);

   std::vector<double> tempData(nPoints);
   int newStartIndex = startDist(random);
   for (int i = 0; i < nPoints; i++)
   {
      tempData[i] = dist(random);
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   // Test Bad Start Index, data should be unchanged
   dataBufferEntry->shiftBuffer(-1);
   for (int i = 0; i < nPoints - 100; i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);

   dataBufferEntry->shiftBuffer(nPoints + 10);
   for (int i = 0; i < nPoints - 100; i++)
      EXPECT_EQ(tempData[i], dataBufferEntry->getBuffer()[i]);

   // Test packing
   dataBufferEntry->shiftBuffer(newStartIndex);

   for (int i = 0; i < nPoints - 1 - newStartIndex; i++)
      EXPECT_EQ(tempData[newStartIndex + i], dataBufferEntry->getBuffer()[i]);
}

TEST_F(YoBufferVariableEntryTest, testGetMax)
{
   std::mt19937 random(6789423U);
   std::uniform_int_distribution<int> dist(0, 499);

   int tempInteger = dist(random) + 11;
   yoDouble->set(tempInteger);
   dataBufferEntry->writeIntoBufferAt(0);
   yoDouble->set(tempInteger + 10);
   dataBufferEntry->writeIntoBufferAt(1);
   yoDouble->set(tempInteger - 10);
   dataBufferEntry->writeIntoBufferAt(2);
   EXPECT_EQ(tempInteger + 10, dataBufferEntry->getUpperBound());
}

TEST_F(YoBufferVariableEntryTest, testGetMin)
{
   std::mt19937 random(213705602U);
   std::uniform_int_distribution<int> dist(0, 499);

   int tempInteger = dist(random) + 11;
   yoDouble->set(tempInteger);
   dataBufferEntry->writeIntoBufferAt(0);
   yoDouble->set(tempInteger + 10);
   dataBufferEntry->writeIntoBufferAt(1);
   yoDouble->set(tempInteger - 10);
   dataBufferEntry->writeIntoBufferAt(2);
   EXPECT_EQ(0, dataBufferEntry->getBounds().getLowerBound());
}

TEST_F(YoBufferVariableEntryTest, testMinMaxWithNaN)
{
   for (int i = 0; i < 100; i++)
   {
      yoDouble->set(std::nan(""));
      dataBufferEntry->writeIntoBufferAt(i);
   }
   EXPECT_EQ(0.0, dataBufferEntry->getBounds().getLowerBound());
   EXPECT_EQ(0.0, dataBufferEntry->getBounds().getUpperBound());
}

TEST_F(YoBufferVariableEntryTest, testMinMaxWithNaN2)
{
   std::mt19937 random(23785U);
   std::uniform_real_distribution<double> dist(0.0, 1.0);

   for (int i = 0; i < 100; i++)
   {
      if (i == 50)
         yoDouble->set(std::nan(""));
      else
         yoDouble->set(dist(random));
      dataBufferEntry->writeIntoBufferAt(i);
   }
   YoBufferBounds bounds = dataBufferEntry->getBounds();
   EXPECT_FALSE(std::isnan(bounds.getLowerBound()));
   EXPECT_FALSE(std::isnan(bounds.getUpperBound()));
   EXPECT_TRUE(bounds.getLowerBound() <= bounds.getUpperBound());
}

TEST_F(YoBufferVariableEntryTest, testResetMinMaxChanged)
{
   std::mt19937 random(90237U);
   std::uniform_int_distribution<int> dist(0, 499);

   int tempInteger = dist(random) + 11;
   yoDouble->set(tempInteger);
   dataBufferEntry->writeIntoBufferAt(0);
   EXPECT_TRUE(dataBufferEntry->haveBoundsChanged());
   dataBufferEntry->resetBoundsChangedFlag();
   EXPECT_FALSE(dataBufferEntry->haveBoundsChanged());
}

// Java's writeBufferAt(value, index) is package-private, reachable from the same-package test.
// This port's writeBufferAt is fully private (see YoBufferVariableEntry's class comment) since C++
// has no package-private equivalent; setting the variable then calling the public
// writeIntoBufferAt(index) achieves the same buffer state through the public API.
TEST_F(YoBufferVariableEntryTest, testSetData)
{
   std::mt19937 random(23987U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);
   std::uniform_int_distribution<int> indexDist(0, nPoints - 1);

   double tempDouble = dist(random);
   int randomIndex = indexDist(random);
   yoDouble->set(tempDouble);
   dataBufferEntry->writeIntoBufferAt(randomIndex);

   EXPECT_EQ(tempDouble, dataBufferEntry->getBuffer()[randomIndex]);
}

TEST_F(YoBufferVariableEntryTest, testGetWindowedData)
{
   std::mt19937 random(37905U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);
   std::uniform_int_distribution<int> indexDist(0, nPoints - 2);

   std::vector<double> tempData(nPoints);
   int randomIndex = indexDist(random);
   for (int i = 0; i < nPoints; i++)
   {
      tempData[i] = dist(random);
      yoDouble->set(tempData[i]);
      dataBufferEntry->writeIntoBufferAt(i);
   }

   std::vector<double> tempDataSubset(nPoints - randomIndex);
   for (std::size_t i = 0; i < tempDataSubset.size(); i++)
      tempDataSubset[i] = tempData[randomIndex + i];

   std::vector<double> windowedData = dataBufferEntry->getBufferWindow(randomIndex, nPoints - randomIndex);

   for (std::size_t i = 0; i < windowedData.size(); i++)
      EXPECT_EQ(tempDataSubset[i], windowedData[i]);
}

TEST_F(YoBufferVariableEntryTest, testEnableAutoScale)
{
   dataBufferEntry->useCustomBounds(true);
   EXPECT_TRUE(dataBufferEntry->isUsingCustomBounds());
   dataBufferEntry->useCustomBounds(false);
   EXPECT_FALSE(dataBufferEntry->isUsingCustomBounds());
}

TEST_F(YoBufferVariableEntryTest, testGetWindowBounds)
{
   std::mt19937 random(74839U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   for (int i = 0; i < nPoints; i++)
   {
      yoDouble->set(dist(random));
      dataBufferEntry->writeIntoBufferAt(i);
   }

   double oldMax = dataBufferEntry->getUpperBound();
   double newMax = oldMax + 100;

   yoDouble->set(newMax);
   dataBufferEntry->writeIntoBufferAt(200);
   yoDouble->set(oldMax);
   dataBufferEntry->writeIntoBufferAt(400);

   EXPECT_EQ(newMax, dataBufferEntry->getWindowUpperBound(150, 250));
   EXPECT_EQ(oldMax, dataBufferEntry->getWindowUpperBound(350, 450));
}

TEST_F(YoBufferVariableEntryTest, testGetMinWithParameters)
{
   std::mt19937 random(751290U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   for (int i = 0; i < nPoints; i++)
   {
      yoDouble->set(dist(random));
      dataBufferEntry->writeIntoBufferAt(i);
   }

   double oldMin = dataBufferEntry->getBounds().getLowerBound();
   double newMin = oldMin - 100;

   yoDouble->set(newMin);
   dataBufferEntry->writeIntoBufferAt(200);
   yoDouble->set(oldMin);
   dataBufferEntry->writeIntoBufferAt(400);

   EXPECT_EQ(newMin, dataBufferEntry->getWindowLowerBound(150, 250));
   EXPECT_EQ(oldMin, dataBufferEntry->getWindowLowerBound(350, 450));
}

TEST_F(YoBufferVariableEntryTest, testThinData)
{
   std::mt19937 random(53290U);
   std::uniform_real_distribution<double> dist(-10000.0, 10000.0);

   for (int i = 0; i < nPoints; i++)
   {
      yoDouble->set(dist(random));
      dataBufferEntry->writeIntoBufferAt(i);
   }

   EXPECT_EQ(dataBufferEntry->getBufferSize(), nPoints);

   int keepEveryNthPoint = 5;
   dataBufferEntry->thinData(keepEveryNthPoint);

   EXPECT_EQ(dataBufferEntry->getBufferSize(), nPoints / keepEveryNthPoint);
}

TEST_F(YoBufferVariableEntryTest, testGetSetInverted)
{
   dataBufferEntry->setInverted(true);
   EXPECT_TRUE(dataBufferEntry->getInverted());

   dataBufferEntry->setInverted(false);
   EXPECT_FALSE(dataBufferEntry->getInverted());
}

TEST_F(YoBufferVariableEntryTest, testGetVariableName)
{
   EXPECT_EQ(dataBufferEntry->getVariableName(), yoDouble->getName());
}

TEST_F(YoBufferVariableEntryTest, testGetFullVariableNameWithNamespace)
{
   EXPECT_EQ(dataBufferEntry->getVariableFullNameString(), yoDouble->getFullNameString());
}
} // namespace
} // namespace ihmc::yovariables::buffer
