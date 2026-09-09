#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ihmc/yovariables/buffer/interfaces/yo_buffer_index_changed_listener.h"
#include "ihmc/yovariables/buffer/interfaces/yo_buffer_processor.h"
#include "ihmc/yovariables/buffer/yo_buffer.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/tools/yo_search_tools.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"
#include "ihmc/yovariables/variable/yo_integer.h"
#include "ihmc/yovariables/variable/yo_variable.h"

namespace ihmc::yovariables::buffer
{
namespace
{
enum class EnumYoVariableTestEnums
{
   ONE,
   TWO
};

class RecordingIndexListener : public interfaces::YoBufferIndexChangedListener
{
public:
   void indexChanged(int) override
   {
      notified = true;
   }

   bool notified = false;
};

class ForwardProcessor : public interfaces::YoBufferProcessor
{
public:
   ForwardProcessor(variable::YoDouble* a, variable::YoDouble* b, variable::YoDouble* c) : a_(a), b_(b), c_(c)
   {
   }

   void process(int, int, int) override
   {
      a_->set(1.0);
      b_->set(2.348);
      c_->set(8.7834);
   }

private:
   variable::YoDouble* a_;
   variable::YoDouble* b_;
   variable::YoDouble* c_;
};

class BackwardProcessor : public interfaces::YoBufferProcessor
{
public:
   BackwardProcessor(variable::YoDouble* a, variable::YoDouble* b, variable::YoDouble* c) : a_(a), b_(b), c_(c)
   {
   }

   bool goForward() const override
   {
      return false;
   }

   void process(int, int, int) override
   {
      a_->set(0.0);
      b_->set(0.0);
      c_->set(0.0);
   }

private:
   variable::YoDouble* a_;
   variable::YoDouble* b_;
   variable::YoDouble* c_;
};

class YoBufferTest : public ::testing::Test
{
protected:
   const int testBufferSize = 100;

   void SetUp() override
   {
      registry = std::make_unique<registry::YoRegistry>("testRegistry");
      yoDouble = std::make_unique<variable::YoDouble>("yoDouble", registry.get());
      yoBoolean = std::make_unique<variable::YoBoolean>("yoBoolean", registry.get());
      yoInteger = std::make_unique<variable::YoInteger>("yoInteger", registry.get());
      yoEnum = std::make_unique<variable::YoEnum<EnumYoVariableTestEnums>>("yoEnum", registry.get());

      a = std::make_unique<variable::YoDouble>("a_arm", registry.get());
      b = std::make_unique<variable::YoDouble>("b_arm", registry.get());
      c = std::make_unique<variable::YoDouble>("c_arm", registry.get());

      aBuffer = std::make_unique<YoBufferVariableEntry>(*a, testBufferSize);
      bBuffer = std::make_unique<YoBufferVariableEntry>(*b, testBufferSize);
      cBuffer = std::make_unique<YoBufferVariableEntry>(*c, testBufferSize);

      dataBuffer = std::make_unique<YoBuffer>(testBufferSize);
   }

   // Registers a_arm/b_arm/c_arm (via aBuffer/bBuffer/cBuffer, which are consumed) with dataBuffer,
   // sets in/out points to the full buffer, then fills each with random data.
   void fillDataBufferWithRandomData(std::mt19937& random)
   {
      dataBuffer->addEntry(std::move(aBuffer));
      dataBuffer->addEntry(std::move(bBuffer));
      dataBuffer->addEntry(std::move(cBuffer));
      dataBuffer->setInOutPointFullBuffer();

      std::uniform_real_distribution<double> dist(0.0, 1.0);
      for (int i = 0; i < testBufferSize; i++)
      {
         a->set(dist(random));
         b->set(dist(random));
         c->set(dist(random));
         dataBuffer->writeIntoBuffer();
         dataBuffer->tickAndReadFromBuffer(1);
      }
   }

   std::unique_ptr<registry::YoRegistry> registry;
   std::unique_ptr<variable::YoEnum<EnumYoVariableTestEnums>> yoEnum;
   std::unique_ptr<variable::YoDouble> yoDouble;
   std::unique_ptr<variable::YoBoolean> yoBoolean;
   std::unique_ptr<variable::YoInteger> yoInteger;
   std::unique_ptr<YoBuffer> dataBuffer;

   std::unique_ptr<variable::YoDouble> a, b, c;
   std::unique_ptr<YoBufferVariableEntry> aBuffer, bBuffer, cBuffer;
};

TEST_F(YoBufferTest, testGetBufferSize)
{
   EXPECT_EQ(dataBuffer->getBufferSize(), testBufferSize);
}

TEST_F(YoBufferTest, testAddAndGetEntry)
{
   auto doubleEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoDouble, testBufferSize);
   auto booleanEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoBoolean, testBufferSize);
   auto integerEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoInteger, testBufferSize);
   auto enumEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoEnum, testBufferSize);

   YoBufferVariableEntry* doubleDataBufferEntryTest = doubleEntryOwned.get();
   YoBufferVariableEntry* booleanDataBufferEntryTest = booleanEntryOwned.get();
   YoBufferVariableEntry* integerDataBufferEntryTest = integerEntryOwned.get();
   YoBufferVariableEntry* enumDataBufferEntryTest = enumEntryOwned.get();

   dataBuffer->addEntry(std::move(doubleEntryOwned));
   dataBuffer->addEntry(std::move(booleanEntryOwned));
   dataBuffer->addEntry(std::move(integerEntryOwned));
   dataBuffer->addEntry(std::move(enumEntryOwned));

   EXPECT_EQ(doubleDataBufferEntryTest, dataBuffer->findVariableEntry("yoDouble"));
   EXPECT_EQ(doubleDataBufferEntryTest, dataBuffer->getEntry(*yoDouble));

   EXPECT_EQ(booleanDataBufferEntryTest, dataBuffer->findVariableEntry("yoBoolean"));
   EXPECT_EQ(booleanDataBufferEntryTest, dataBuffer->getEntry(*yoBoolean));

   EXPECT_EQ(integerDataBufferEntryTest, dataBuffer->findVariableEntry("yoInteger"));
   EXPECT_EQ(integerDataBufferEntryTest, dataBuffer->getEntry(*yoInteger));

   EXPECT_EQ(enumDataBufferEntryTest, dataBuffer->findVariableEntry("yoEnum"));
   EXPECT_EQ(enumDataBufferEntryTest, dataBuffer->getEntry(*yoEnum));
}

TEST_F(YoBufferTest, testAddNewEntry)
{
   dataBuffer->addVariable(*yoDouble);
   dataBuffer->addVariable(*yoBoolean);
   dataBuffer->addVariable(*yoInteger);
   dataBuffer->addVariable(*yoEnum);

   YoBufferVariableEntry doubleDataBufferEntryTest(*yoDouble, testBufferSize);
   YoBufferVariableEntry booleanDataBufferEntryTest(*yoBoolean, testBufferSize);
   YoBufferVariableEntry integerDataBufferEntryTest(*yoInteger, testBufferSize);
   YoBufferVariableEntry enumDataBufferEntryTest(*yoEnum, testBufferSize);

   EXPECT_EQ(&doubleDataBufferEntryTest.getVariable(), &dataBuffer->getEntry(*yoDouble)->getVariable());
   EXPECT_EQ(&booleanDataBufferEntryTest.getVariable(), &dataBuffer->getEntry(*yoBoolean)->getVariable());
   EXPECT_EQ(&integerDataBufferEntryTest.getVariable(), &dataBuffer->getEntry(*yoInteger)->getVariable());
   EXPECT_EQ(&enumDataBufferEntryTest.getVariable(), &dataBuffer->getEntry(*yoEnum)->getVariable());
}

TEST_F(YoBufferTest, testAddVariable)
{
   dataBuffer->addVariable(*yoDouble);
   dataBuffer->addVariable(*yoBoolean);
   dataBuffer->addVariable(*yoInteger);
   dataBuffer->addVariable(*yoEnum);

   EXPECT_EQ(yoDouble.get(), &dataBuffer->getEntry(*yoDouble)->getVariable());
   EXPECT_EQ(yoBoolean.get(), &dataBuffer->getEntry(*yoBoolean)->getVariable());
   EXPECT_EQ(yoInteger.get(), &dataBuffer->getEntry(*yoInteger)->getVariable());
   EXPECT_EQ(yoEnum.get(), &dataBuffer->getEntry(*yoEnum)->getVariable());
}

TEST_F(YoBufferTest, testAddVariableWithArrayList)
{
   std::vector<variable::YoVariable*> arrayListToBeAdded{yoDouble.get(), yoBoolean.get(), yoInteger.get(), yoEnum.get()};

   dataBuffer->addVariables(arrayListToBeAdded);

   EXPECT_EQ(yoDouble.get(), &dataBuffer->getEntry(*yoDouble)->getVariable());
   EXPECT_EQ(yoBoolean.get(), &dataBuffer->getEntry(*yoBoolean)->getVariable());
   EXPECT_EQ(yoInteger.get(), &dataBuffer->getEntry(*yoInteger)->getVariable());
   EXPECT_EQ(yoEnum.get(), &dataBuffer->getEntry(*yoEnum)->getVariable());
}

// Just adds variables and checks nothing throws - matches the original, which has no assertions.
TEST_F(YoBufferTest, testGetVariablesThatStartWith)
{
   variable::YoDouble yoVariable1("doy", registry.get());
   variable::YoDouble yoVariable2("Dog", registry.get());
   variable::YoDouble yoVariable3("bar", registry.get());

   dataBuffer->addVariable(*yoDouble);
   dataBuffer->addVariable(*yoBoolean);
   dataBuffer->addVariable(*yoInteger);
   dataBuffer->addVariable(*yoEnum);
   dataBuffer->addVariable(yoVariable1);
   dataBuffer->addVariable(yoVariable2);
   dataBuffer->addVariable(yoVariable3);
}

TEST_F(YoBufferTest, testGetEntries)
{
   auto doubleEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoDouble, testBufferSize);
   auto booleanEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoBoolean, testBufferSize);
   auto integerEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoInteger, testBufferSize);
   auto enumEntryOwned = std::make_unique<YoBufferVariableEntry>(*yoEnum, testBufferSize);

   std::vector<YoBufferVariableEntry*> expectedDataEntries{doubleEntryOwned.get(), booleanEntryOwned.get(), integerEntryOwned.get(), enumEntryOwned.get()};

   dataBuffer->addEntry(std::move(doubleEntryOwned));
   dataBuffer->addEntry(std::move(booleanEntryOwned));
   dataBuffer->addEntry(std::move(integerEntryOwned));
   dataBuffer->addEntry(std::move(enumEntryOwned));

   const std::vector<std::unique_ptr<YoBufferVariableEntry>>& actualDataEntries = dataBuffer->getEntries();
   ASSERT_EQ(actualDataEntries.size(), expectedDataEntries.size());
   for (std::size_t i = 0; i < expectedDataEntries.size(); i++)
      EXPECT_EQ(actualDataEntries[i].get(), expectedDataEntries[i]);
}

TEST_F(YoBufferTest, testGetVariables)
{
   dataBuffer->addVariable(*yoDouble);
   dataBuffer->addVariable(*yoBoolean);
   dataBuffer->addVariable(*yoInteger);
   dataBuffer->addVariable(*yoEnum);

   std::vector<variable::YoVariable*> expectedArrayOfVariables{yoDouble.get(), yoBoolean.get(), yoInteger.get(), yoEnum.get()};
   std::vector<variable::YoVariable*> actualArrayOfVariables = dataBuffer->getVariables();

   for (variable::YoVariable* actual : actualArrayOfVariables)
      EXPECT_NE(std::find(expectedArrayOfVariables.begin(), expectedArrayOfVariables.end(), actual), expectedArrayOfVariables.end());
}

TEST_F(YoBufferTest, testEmptyBufferIncreaseBufferSize)
{
   int originalBufferSize = dataBuffer->getBufferSize();
   int newBufferSize = originalBufferSize * 2;

   dataBuffer->resizeBuffer(newBufferSize);
   EXPECT_EQ(newBufferSize, dataBuffer->getBufferSize());
}

TEST_F(YoBufferTest, testEmptyBufferDecreaseBufferSize)
{
   int originalBufferSize = dataBuffer->getBufferSize();
   int newBufferSize = originalBufferSize / 2;

   dataBuffer->resizeBuffer(newBufferSize);
   EXPECT_EQ(newBufferSize, dataBuffer->getBufferSize());
}

TEST_F(YoBufferTest, testEnlargeBufferSize)
{
   dataBuffer->addEntry(std::make_unique<YoBufferVariableEntry>(*yoDouble, testBufferSize));

   int originalBufferSize = dataBuffer->getBufferSize();
   int newBufferSize = originalBufferSize * 2;

   dataBuffer->resizeBuffer(newBufferSize);
   EXPECT_EQ(newBufferSize, dataBuffer->getBufferSize());
}

TEST_F(YoBufferTest, testDecreaseBufferSize)
{
   dataBuffer->addEntry(std::make_unique<YoBufferVariableEntry>(*yoDouble, testBufferSize));

   int originalBufferSize = dataBuffer->getBufferSize();
   int newBufferSize = originalBufferSize / 2;

   dataBuffer->resizeBuffer(newBufferSize);
   EXPECT_EQ(newBufferSize, dataBuffer->getBufferSize());
}

TEST_F(YoBufferTest, testTick)
{
   int numberOfTicksAndUpdates = 20;
   for (int i = 0; i < numberOfTicksAndUpdates; i++)
      dataBuffer->tickAndWriteIntoBuffer();

   dataBuffer->gotoInPoint();

   int expectedIndex = 0;
   while (dataBuffer->getCurrentIndex() < dataBuffer->getBufferInOutLength() - 1)
   {
      EXPECT_EQ(expectedIndex, dataBuffer->getCurrentIndex());
      bool rolledOver = dataBuffer->tickAndReadFromBuffer(1);
      EXPECT_FALSE(rolledOver);
      expectedIndex++;
   }

   bool rolledOver = dataBuffer->tickAndReadFromBuffer(1);
   EXPECT_TRUE(rolledOver);
   expectedIndex = 0;
   EXPECT_EQ(expectedIndex, dataBuffer->getCurrentIndex());

   rolledOver = dataBuffer->tickAndReadFromBuffer(1);
   EXPECT_FALSE(rolledOver);
   expectedIndex = 1;
   EXPECT_EQ(expectedIndex, dataBuffer->getCurrentIndex());
}

TEST_F(YoBufferTest, testCropBuffer)
{
   // Test bugfix: cropping twice should not throw an arithmetic exception.
   dataBuffer->cropBuffer();
   int bufferSize = dataBuffer->getBufferSize();
   dataBuffer->cropBuffer();
   EXPECT_EQ(bufferSize, dataBuffer->getBufferSize());
}

TEST_F(YoBufferTest, testIsIndexBetweenInAndOutPoint)
{
   EXPECT_EQ(0, dataBuffer->getCurrentIndex());
   EXPECT_EQ(0, dataBuffer->getInPoint());
   EXPECT_EQ(0, dataBuffer->getOutPoint());
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(0));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(1));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(-1));

   dataBuffer->tickAndWriteIntoBuffer();
   EXPECT_EQ(1, dataBuffer->getCurrentIndex());
   EXPECT_EQ(0, dataBuffer->getInPoint());
   EXPECT_EQ(1, dataBuffer->getOutPoint());
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(0));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(1));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(-1));

   dataBuffer->tickAndWriteIntoBuffer();
   EXPECT_EQ(2, dataBuffer->getCurrentIndex());
   EXPECT_EQ(0, dataBuffer->getInPoint());
   EXPECT_EQ(2, dataBuffer->getOutPoint());
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(0));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(1));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(2));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(3));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(-1));

   int numTicks = 20;
   for (int i = 0; i < numTicks; i++)
      dataBuffer->tickAndWriteIntoBuffer();
   EXPECT_EQ(dataBuffer->getOutPoint(), dataBuffer->getCurrentIndex());
   EXPECT_EQ(0, dataBuffer->getInPoint());

   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex() - 1));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex()));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex() + 1));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(dataBuffer->getInPoint() - 1));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getInPoint()));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getInPoint() + 1));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getOutPoint() - 1));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getOutPoint()));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(dataBuffer->getOutPoint() + 1));

   dataBuffer->cropBuffer();
   dataBuffer->gotoOutPoint();

   EXPECT_EQ(dataBuffer->getOutPoint(), dataBuffer->getCurrentIndex());
   EXPECT_EQ(0, dataBuffer->getInPoint());

   numTicks = 7;

   for (int i = 0; i < numTicks; i++)
      dataBuffer->tickAndWriteIntoBuffer();

   EXPECT_EQ(dataBuffer->getOutPoint(), dataBuffer->getCurrentIndex());
   EXPECT_EQ(numTicks - 1, dataBuffer->getOutPoint());
   EXPECT_EQ(numTicks, dataBuffer->getInPoint());

   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex() - 1));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex()));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex() + 1));

   dataBuffer->tickAndWriteIntoBuffer();
   EXPECT_EQ(dataBuffer->getOutPoint(), dataBuffer->getCurrentIndex());
   EXPECT_EQ(dataBuffer->getOutPoint(), dataBuffer->getInPoint() - 1);

   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex() - 1));
   EXPECT_TRUE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex()));
   EXPECT_FALSE(dataBuffer->isIndexBetweenBounds(dataBuffer->getCurrentIndex() + 1));
}

TEST_F(YoBufferTest, testSetLockIndex)
{
   EXPECT_FALSE(dataBuffer->isIndexLocked());
   dataBuffer->setLockIndex(true);
   EXPECT_TRUE(dataBuffer->isIndexLocked());
   dataBuffer->setLockIndex(false);
   EXPECT_FALSE(dataBuffer->isIndexLocked());
}

TEST_F(YoBufferTest, testGetVars)
{
   dataBuffer->addEntry(std::move(aBuffer));
   dataBuffer->addEntry(std::move(bBuffer));
   dataBuffer->addEntry(std::move(cBuffer));

   std::vector<std::string> varNames{"a_arm", "b_arm", "c_arm"};
   std::vector<std::string> aNames{"a_arm"};

   std::vector<variable::YoVariable*> justNames;
   for (const std::string& varName : varNames)
   {
      std::vector<variable::YoVariable*> found = dataBuffer->findVariables(varName);
      justNames.insert(justNames.end(), found.begin(), found.end());
   }

   EXPECT_NE(std::find(justNames.begin(), justNames.end(), a.get()), justNames.end());
   EXPECT_NE(std::find(justNames.begin(), justNames.end(), b.get()), justNames.end());
   EXPECT_NE(std::find(justNames.begin(), justNames.end(), c.get()), justNames.end());

   std::vector<variable::YoVariable*> justA;
   for (const std::string& varName : aNames)
   {
      std::vector<variable::YoVariable*> found = dataBuffer->findVariables(varName);
      justA.insert(justA.end(), found.begin(), found.end());
   }

   EXPECT_NE(std::find(justA.begin(), justA.end(), a.get()), justA.end());
   EXPECT_EQ(std::find(justA.begin(), justA.end(), b.get()), justA.end());
   EXPECT_EQ(std::find(justA.begin(), justA.end(), c.get()), justA.end());

   std::vector<variable::YoVariable*> justRegExp = dataBuffer->filterVariables(tools::regularExpressionFilter({".*"}));

   EXPECT_NE(std::find(justRegExp.begin(), justRegExp.end(), a.get()), justRegExp.end());
   EXPECT_NE(std::find(justRegExp.begin(), justRegExp.end(), b.get()), justRegExp.end());
   EXPECT_NE(std::find(justRegExp.begin(), justRegExp.end(), c.get()), justRegExp.end());

   std::vector<variable::YoVariable*> cRegExp = dataBuffer->filterVariables(tools::regularExpressionFilter({"c.*"}));

   EXPECT_EQ(std::find(cRegExp.begin(), cRegExp.end(), a.get()), cRegExp.end());
   EXPECT_EQ(std::find(cRegExp.begin(), cRegExp.end(), b.get()), cRegExp.end());
   EXPECT_NE(std::find(cRegExp.begin(), cRegExp.end(), c.get()), cRegExp.end());
}

TEST_F(YoBufferTest, testCloseAndDispose)
{
   dataBuffer->addEntry(std::move(aBuffer));
   dataBuffer->addEntry(std::move(bBuffer));
   dataBuffer->addEntry(std::move(cBuffer));
   EXPECT_EQ(dataBuffer->getEntries().size(), 3u);
   dataBuffer->clear();
   EXPECT_TRUE(dataBuffer->getEntries().empty());
   EXPECT_EQ(dataBuffer->getCurrentIndex(), 0);
}

TEST_F(YoBufferTest, testCopyValuesThrough)
{
   std::mt19937 random(574893U);
   fillDataBufferWithRandomData(random);

   const std::vector<std::unique_ptr<YoBufferVariableEntry>>& entries = dataBuffer->getEntries();

   // check that each point for each entry is filled with random data
   for (const auto& dataBufferEntry : entries)
   {
      std::vector<double> data = dataBufferEntry->getBuffer();
      for (std::size_t j = 0; j + 1 < data.size(); j++)
         EXPECT_NE(data[j], 0.0); // assuming that 0.0 wasn't randomly generated
   }

   // method being tested: replace each data point in each entry with the current value of the
   // YoVariable assigned to that entry
   dataBuffer->fillBuffer();

   // each point for each entry should now equal the current value of the entry's YoVariable
   for (const auto& dataBufferEntry : entries)
   {
      variable::YoVariable& variable = dataBufferEntry->getVariable();
      std::vector<double> data = dataBufferEntry->getBuffer();
      for (std::size_t j = 0; j + 1 < data.size(); j++)
         EXPECT_EQ(data[j], variable.getValueAsDouble());
   }
}

TEST_F(YoBufferTest, testPackDataWithInvalidStartPoint)
{
   std::mt19937 random(27093U);
   fillDataBufferWithRandomData(random);

   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);

   dataBuffer->shiftBuffer(-1);
   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);

   dataBuffer->shiftBuffer(testBufferSize);
   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);
}

TEST_F(YoBufferTest, testPackData)
{
   const int kTestIterations = 1000;
   std::mt19937 random(209390U);
   fillDataBufferWithRandomData(random);

   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);

   std::uniform_int_distribution<int> indexDist(0, testBufferSize - 1);

   for (int i = 0; i < kTestIterations; i++)
   {
      YoBuffer dataBufferClone(*dataBuffer);
      int newIndex = indexDist(random);
      dataBuffer->setCurrentIndex(newIndex);
      int newStartLocation = indexDist(random);
      dataBuffer->shiftBuffer(newStartLocation);

      const std::vector<std::unique_ptr<YoBufferVariableEntry>>& entries = dataBuffer->getEntries();
      const std::vector<std::unique_ptr<YoBufferVariableEntry>>& entriesClone = dataBufferClone.getEntries();
      for (std::size_t j = 0; j < entries.size(); j++)
      {
         std::vector<double> data = entries[j]->getBuffer();
         std::vector<double> dataClone = entriesClone[j]->getBuffer();

         for (std::size_t k = 0; k < data.size(); k++)
            EXPECT_EQ(dataClone[k], data[(k + testBufferSize - newStartLocation) % testBufferSize]);

         if (newStartLocation >= newIndex)
            EXPECT_EQ(dataBuffer->getCurrentIndex(), 0);
         else
            EXPECT_EQ(dataBuffer->getCurrentIndex(), newIndex - newStartLocation);
         EXPECT_EQ(dataBuffer->getInPoint(), 0);
         EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1 - newStartLocation);
      }

      dataBuffer->setInOutPointFullBuffer();
   }
}

TEST_F(YoBufferTest, testCheckIfDataIsEqual)
{
   YoBuffer localDataBuffer(0);
   YoBuffer otherDataBuffer(0);

   EXPECT_TRUE(localDataBuffer.epsilonEquals(otherDataBuffer, 1e-6));

   registry::YoRegistry dataBufferRegistry("dataBufferRegistry");
   variable::YoDouble dataBufferYoDouble("dataBufferYoDouble", &dataBufferRegistry);
   localDataBuffer.addEntry(std::make_unique<YoBufferVariableEntry>(dataBufferYoDouble, 0));

   EXPECT_FALSE(localDataBuffer.epsilonEquals(otherDataBuffer, 1));

   registry::YoRegistry otherDataBufferRegistry("otherDataBufferRegistry");
   variable::YoDouble otherDataBufferYoDouble("otherDataBufferYoDouble", &otherDataBufferRegistry);
   otherDataBuffer.addEntry(std::make_unique<YoBufferVariableEntry>(otherDataBufferYoDouble, 0));

   EXPECT_FALSE(localDataBuffer.epsilonEquals(otherDataBuffer, 1e-6));

   YoBuffer freshDataBuffer(testBufferSize);
   YoBuffer freshOtherDataBuffer(testBufferSize);

   freshDataBuffer.addVariable(dataBufferYoDouble);
   freshOtherDataBuffer.addVariable(dataBufferYoDouble);

   int numberOfTicks = 5;
   for (int i = 0; i < numberOfTicks; i++)
   {
      dataBufferYoDouble.set(1.0);
      freshDataBuffer.tickAndWriteIntoBuffer();

      dataBufferYoDouble.set(0.0);
      freshOtherDataBuffer.tickAndWriteIntoBuffer();
   }

   EXPECT_FALSE(freshDataBuffer.epsilonEquals(freshOtherDataBuffer, 1e-6));
}

TEST_F(YoBufferTest, testCloneDataBuffer)
{
   std::mt19937 random(19824U);
   fillDataBufferWithRandomData(random);
   YoBuffer dataBufferClone(*dataBuffer);

   EXPECT_TRUE(dataBuffer->epsilonEquals(dataBufferClone, 1e-6));
}

TEST_F(YoBufferTest, testCutDataWithInvalidStartAndEnd)
{
   std::mt19937 random(6543897U);
   fillDataBufferWithRandomData(random);

   EXPECT_EQ(dataBuffer->getCurrentIndex(), 0);
   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);

   dataBuffer->cutBuffer(-1, testBufferSize / 2);

   EXPECT_EQ(dataBuffer->getCurrentIndex(), 0);
   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);

   dataBuffer->cutBuffer(testBufferSize / 2, testBufferSize + 1);

   EXPECT_EQ(dataBuffer->getCurrentIndex(), 0);
   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);
}

TEST_F(YoBufferTest, testCutDataOfEntireBuffer)
{
   std::mt19937 random(27489U);
   fillDataBufferWithRandomData(random);

   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);

   // the no-arg overload should use the in/out points for the start and end of the region to cut;
   // this should cut the entire buffer and effectively erase all of its data
   dataBuffer->cutBuffer();

   // the length of the buffer shouldn't change - true when the entire buffer is cut
   EXPECT_EQ(dataBuffer->getBufferSize(), testBufferSize);

   for (const auto& entry : dataBuffer->getEntries())
      for (double d : entry->getBuffer())
         EXPECT_EQ(d, 0.0);
}

TEST_F(YoBufferTest, testCutData)
{
   const int kTestIterations = 1000;
   std::mt19937 random(345890U);
   fillDataBufferWithRandomData(random);

   EXPECT_EQ(dataBuffer->getInPoint(), 0);
   EXPECT_EQ(dataBuffer->getOutPoint(), testBufferSize - 1);

   YoBuffer unmodifiedDataBuffer(*dataBuffer);
   std::uniform_int_distribution<int> indexDist(0, testBufferSize - 1);

   for (int i = 0; i < kTestIterations; i++)
   {
      int start = indexDist(random);
      int end = indexDist(random);

      dataBuffer->cutBuffer(start, end);

      const std::vector<std::unique_ptr<YoBufferVariableEntry>>& entries = dataBuffer->getEntries();
      const std::vector<std::unique_ptr<YoBufferVariableEntry>>& unmodifiedEntries = unmodifiedDataBuffer.getEntries();

      for (std::size_t j = 0; j < entries.size(); j++)
      {
         std::vector<double> cutData = entries[j]->getBuffer();
         std::vector<double> unmodifiedData = unmodifiedEntries[j]->getBuffer();

         for (std::size_t k = 0; k < cutData.size(); k++)
         {
            if (start < end)
            {
               EXPECT_EQ(dataBuffer->getBufferSize(), testBufferSize - (end - start + 1));
               double dataFromCutEntry = cutData[k];
               std::size_t indexInUnmodifiedEntry = static_cast<int>(k) < start ? k : static_cast<std::size_t>(end + static_cast<int>(k) - start + 1);
               EXPECT_EQ(dataFromCutEntry, unmodifiedData[indexInUnmodifiedEntry]);
            }
            else if (start > end)
            {
               EXPECT_EQ(dataBuffer->getBufferSize(), testBufferSize);
               EXPECT_EQ(cutData[k], unmodifiedData[k]);
            }
            else
            {
               EXPECT_EQ(dataBuffer->getBufferSize(), testBufferSize - 1);
               double dataFromCutEntry = cutData[k];
               std::size_t indexInUnmodifiedEntry = static_cast<int>(k) < start ? k : k + 1;
               EXPECT_EQ(dataFromCutEntry, unmodifiedData[indexInUnmodifiedEntry]);
            }
         }
      }

      dataBuffer = std::make_unique<YoBuffer>(unmodifiedDataBuffer);
   }
}

TEST_F(YoBufferTest, testThinData)
{
   const int kTestIterations = 1000;
   std::mt19937 random(246370U);
   fillDataBufferWithRandomData(random);

   YoBuffer unmodifiedDataBuffer(*dataBuffer);
   std::uniform_int_distribution<int> nDist(1, testBufferSize - 1);

   for (int i = 0; i < kTestIterations; i++)
   {
      int keepEveryNthPoint = nDist(random);
      dataBuffer->thinData(keepEveryNthPoint);

      const std::vector<std::unique_ptr<YoBufferVariableEntry>>& entries = dataBuffer->getEntries();
      const std::vector<std::unique_ptr<YoBufferVariableEntry>>& unmodifiedEntries = unmodifiedDataBuffer.getEntries();

      for (std::size_t j = 0; j < entries.size(); j++)
      {
         std::vector<double> thinnedEntryData = entries[j]->getBuffer();
         std::vector<double> unmodifiedEntryData = unmodifiedEntries[j]->getBuffer();

         for (std::size_t k = 0; k < thinnedEntryData.size(); k++)
         {
            if (keepEveryNthPoint < testBufferSize / 2)
            {
               EXPECT_EQ(entries[j]->getBufferSize(), testBufferSize / keepEveryNthPoint);
               EXPECT_EQ(thinnedEntryData[k], unmodifiedEntryData[k * static_cast<std::size_t>(keepEveryNthPoint)]);
            }
            else
            {
               EXPECT_EQ(entries[j]->getBufferSize(), testBufferSize);
               EXPECT_EQ(thinnedEntryData[k], unmodifiedEntryData[k]);
            }
         }
      }

      dataBuffer = std::make_unique<YoBuffer>(unmodifiedDataBuffer);
   }
}

TEST_F(YoBufferTest, testAttachIndexChangedListener)
{
   RecordingIndexListener listener;

   dataBuffer->addListener(&listener);

   EXPECT_FALSE(listener.notified);

   dataBuffer->tickAndWriteIntoBuffer();

   EXPECT_TRUE(listener.notified);
}

TEST_F(YoBufferTest, testApplyDataProcessingFunction)
{
   std::mt19937 random(74523U);
   fillDataBufferWithRandomData(random);

   ForwardProcessor forwardDataProcessingFunction(a.get(), b.get(), c.get());

   EXPECT_FALSE(a->getDoubleValue() == 1.0);
   EXPECT_FALSE(b->getDoubleValue() == 2.348);
   EXPECT_FALSE(c->getDoubleValue() == 8.7834);

   dataBuffer->applyProcessor(forwardDataProcessingFunction);

   EXPECT_TRUE(a->getDoubleValue() == 1.0);
   EXPECT_TRUE(b->getDoubleValue() == 2.348);
   EXPECT_TRUE(c->getDoubleValue() == 8.7834);

   dataBuffer->setCurrentIndex(dataBuffer->getOutPoint());

   BackwardProcessor backwardsDataProcessingFunction(a.get(), b.get(), c.get());

   EXPECT_FALSE(a->getDoubleValue() == 0.0);
   EXPECT_FALSE(b->getDoubleValue() == 0.0);
   EXPECT_FALSE(c->getDoubleValue() == 0.0);

   dataBuffer->applyProcessor(backwardsDataProcessingFunction);

   EXPECT_TRUE(a->getDoubleValue() == 0.0);
   EXPECT_TRUE(b->getDoubleValue() == 0.0);
   EXPECT_TRUE(c->getDoubleValue() == 0.0);
}

TEST_F(YoBufferTest, testToggleKeyPointMode)
{
   bool keyPointModeToggled = dataBuffer->getKeyPointsHandler().areKeyPointsEnabled();

   dataBuffer->getKeyPointsHandler().toggleKeyPoints();

   EXPECT_NE(keyPointModeToggled, dataBuffer->getKeyPointsHandler().areKeyPointsEnabled());

   dataBuffer->getKeyPointsHandler().toggleKeyPoints();

   EXPECT_EQ(keyPointModeToggled, dataBuffer->getKeyPointsHandler().areKeyPointsEnabled());
}

TEST_F(YoBufferTest, testGetSetTimeVariable)
{
   std::string timeVariableName = "time";
   variable::YoDouble time(timeVariableName, registry.get());

   dataBuffer->addVariable(time);
   dataBuffer->setTimeVariableName(timeVariableName);

   int numberOfTicks = 5;
   std::vector<double> timeData(numberOfTicks);

   for (int i = 0; i < numberOfTicks; i++)
   {
      time.set(i);
      timeData[i] = i;
      dataBuffer->tickAndWriteIntoBuffer();
   }

   EXPECT_EQ(dataBuffer->getTimeVariableName(), timeVariableName);

   EXPECT_EQ(static_cast<int>(timeData.size()), dataBuffer->getCurrentIndex());

   std::vector<double> dataBufferTimeData = dataBuffer->getTimeBuffer();
   for (std::size_t i = 0; i < timeData.size(); i++)
   {
      // We have to add 1 to the dataBufferTimeData index because the time variable already has a
      // value of 0.0 when tickAndWriteIntoBuffer is called the first time.
      EXPECT_EQ(timeData[i], dataBufferTimeData[i + 1]);
   }
}
} // namespace
} // namespace ihmc::yovariables::buffer
