#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ihmc/yovariables/filters/backlash_compensating_velocity_yo_variable.h"
#include "ihmc/yovariables/filters/filtered_finite_difference_yo_variable.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_double.h"

namespace ihmc::yovariables::filters
{
namespace
{
constexpr double kEpsilon = 1e-8;

TEST(BacklashCompensatingVelocityYoVariableTest, testWithoutBacklashOrFiltering1)
{
   std::mt19937 rand(1798U);
   std::uniform_real_distribution<double> dtDist(1e-8, 1.0);
   std::uniform_real_distribution<double> posDist(-100.0, 100.0);

   registry::YoRegistry registry("blop");
   variable::YoDouble alphaVariable("alpha", &registry);
   double dt = dtDist(rand);
   variable::YoDouble slopTime("slop", &registry);
   BacklashCompensatingVelocityYoVariable unprocessed("", "", alphaVariable, dt, slopTime, &registry);

   double rawPosition = 0.0, rawPositionPrevValue = 0.0;
   unprocessed.update(rawPosition);

   for (int i = 0; i < 1000; i++)
   {
      rawPosition = posDist(rand);
      unprocessed.update(rawPosition);

      double rawVelocity = (rawPosition - rawPositionPrevValue) / dt;

      EXPECT_NEAR(rawVelocity, unprocessed.getDoubleValue(), kEpsilon);

      rawPositionPrevValue = rawPosition;
   }
}

TEST(BacklashCompensatingVelocityYoVariableTest, testWithoutBacklashOrFiltering2)
{
   std::mt19937 rand(1798U);
   std::uniform_real_distribution<double> dtDist(1e-8, 1.0);
   std::uniform_real_distribution<double> posDist(-100.0, 100.0);

   registry::YoRegistry registry("blop");
   variable::YoDouble alphaVariable("alpha", &registry);
   double dt = dtDist(rand);
   variable::YoDouble slopTime("slop", &registry);
   variable::YoDouble rawPosition("rawPosition", &registry);
   BacklashCompensatingVelocityYoVariable unprocessed("", "", alphaVariable, dt, slopTime, &registry, &rawPosition);

   double rawPositionPrevValue = 0.0;
   unprocessed.update();

   for (int i = 0; i < 1000; i++)
   {
      rawPosition.set(posDist(rand));
      unprocessed.update();

      double rawVelocity = (rawPosition.getDoubleValue() - rawPositionPrevValue) / dt;

      EXPECT_NEAR(rawVelocity, unprocessed.getDoubleValue(), kEpsilon);

      rawPositionPrevValue = rawPosition.getDoubleValue();
   }
}

TEST(BacklashCompensatingVelocityYoVariableTest, testWithoutBacklash1)
{
   std::mt19937 rand(1798U);
   std::uniform_real_distribution<double> dtDist(1e-8, 1.0);
   std::uniform_real_distribution<double> alphaDist(0.1, 1.0);
   std::uniform_real_distribution<double> posDist(-100.0, 100.0);

   registry::YoRegistry registry("blop");
   variable::YoDouble alphaVariable("alpha", &registry);
   alphaVariable.set(alphaDist(rand));
   double dt = dtDist(rand);
   variable::YoDouble slopTime("slop", &registry);
   variable::YoDouble rawPosition("rawPosition", &registry);
   FilteredFiniteDifferenceYoVariable filtVelocity("filtVelocity", "", alphaVariable, dt, &registry, &rawPosition);
   BacklashCompensatingVelocityYoVariable filteredOnly("", "", alphaVariable, dt, slopTime, &registry);

   filtVelocity.update();
   filteredOnly.update(rawPosition.getDoubleValue());

   for (int i = 0; i < 1000; i++)
   {
      alphaVariable.set(alphaDist(rand));
      rawPosition.set(posDist(rand));
      filtVelocity.update();
      filteredOnly.update(rawPosition.getDoubleValue());

      EXPECT_NEAR(filtVelocity.getDoubleValue(), filteredOnly.getDoubleValue(), kEpsilon);
   }
}

TEST(BacklashCompensatingVelocityYoVariableTest, testWithoutBacklash2)
{
   std::mt19937 rand(1798U);
   std::uniform_real_distribution<double> dtDist(1e-8, 1.0);
   std::uniform_real_distribution<double> alpha0Dist(0.0, 1.0);
   std::uniform_real_distribution<double> alphaDist(0.1, 1.0);
   std::uniform_real_distribution<double> posDist(-100.0, 100.0);

   registry::YoRegistry registry("blop");
   variable::YoDouble alphaVariable("alpha", &registry);
   alphaVariable.set(alpha0Dist(rand));
   double dt = dtDist(rand);
   variable::YoDouble slopTime("slop", &registry);
   variable::YoDouble rawPosition("rawPosition", &registry);
   FilteredFiniteDifferenceYoVariable filtVelocity("filtVelocity", "", alphaVariable, dt, &registry, &rawPosition);
   BacklashCompensatingVelocityYoVariable filteredOnly("", "", alphaVariable, dt, slopTime, &registry, &rawPosition);

   filtVelocity.update();
   filteredOnly.update();

   for (int i = 0; i < 1000; i++)
   {
      alphaVariable.set(alphaDist(rand));
      rawPosition.set(posDist(rand));
      filtVelocity.update();
      filteredOnly.update();

      EXPECT_NEAR(filtVelocity.getDoubleValue(), filteredOnly.getDoubleValue(), kEpsilon);
   }
}

TEST(BacklashCompensatingVelocityYoVariableTest, testVelocityPositiveWithoutCrossingZero2)
{
   std::mt19937 rand(1798U);
   std::uniform_real_distribution<double> dtDist(1e-8, 1.0);
   std::uniform_real_distribution<double> alpha0Dist(0.0, 1.0);
   std::uniform_real_distribution<double> alphaDist(0.1, 1.0);
   std::uniform_real_distribution<double> slopDist(0.0, 10.0);
   std::uniform_real_distribution<double> posDeltaDist(0.0, 101.0);

   registry::YoRegistry registry("blop");
   variable::YoDouble alphaVariable("alpha", &registry);
   alphaVariable.set(alpha0Dist(rand));
   double dt = dtDist(rand);
   variable::YoDouble slopTime("slop", &registry);
   variable::YoDouble rawPosition("rawPosition", &registry);
   FilteredFiniteDifferenceYoVariable filtVelocity("filtVelocity", "", alphaVariable, dt, &registry, &rawPosition);
   BacklashCompensatingVelocityYoVariable backlashAndFiltered("", "", alphaVariable, dt, slopTime, &registry, &rawPosition);

   filtVelocity.update();
   backlashAndFiltered.update();

   // In this test, the position is only increasing, so there should be no backlash filtering that gets applied.

   for (int i = 0; i < 10000; i++)
   {
      slopTime.set(slopDist(rand));
      alphaVariable.set(alphaDist(rand));
      rawPosition.add(posDeltaDist(rand));
      filtVelocity.update();
      backlashAndFiltered.update();

      EXPECT_NEAR(filtVelocity.getDoubleValue(), backlashAndFiltered.getDoubleValue(), kEpsilon);
   }
}

TEST(BacklashCompensatingVelocityYoVariableTest, testVelocityNegativeWithoutCrossingZero2)
{
   std::mt19937 rand(1798U);
   std::uniform_real_distribution<double> dtDist(1e-8, 1.0);
   std::uniform_real_distribution<double> alpha0Dist(0.0, 1.0);
   std::uniform_real_distribution<double> slopDist(0.0, 100.0);
   std::uniform_real_distribution<double> posDeltaDist(0.0, 101.0);

   registry::YoRegistry registry("blop");
   variable::YoDouble alphaVariable("alpha", &registry);
   alphaVariable.set(alpha0Dist(rand));
   double dt = dtDist(rand);
   variable::YoDouble slopTime("slop", &registry);
   variable::YoDouble rawPosition("rawPosition", &registry);
   FilteredFiniteDifferenceYoVariable filtVelocity("filtVelocity", "", alphaVariable, dt, &registry, &rawPosition);
   BacklashCompensatingVelocityYoVariable backlashAndFiltered("", "", alphaVariable, dt, slopTime, &registry, &rawPosition);

   filtVelocity.update();
   backlashAndFiltered.update();

   for (int i = 0; i < 1000; i++)
   {
      slopTime.set(slopDist(rand));
      alphaVariable.set(alpha0Dist(rand));
      rawPosition.sub(posDeltaDist(rand));
      filtVelocity.update();
      backlashAndFiltered.update();

      EXPECT_NEAR(filtVelocity.getDoubleValue(), backlashAndFiltered.getDoubleValue(), kEpsilon);
   }
}

TEST(BacklashCompensatingVelocityYoVariableTest, testNoisySignalAndMakeSureVelocityHasSignalContent)
{
   std::mt19937 random(1798U);

   registry::YoRegistry registry("Registry");
   variable::YoDouble alphaVariable("alpha", &registry);
   variable::YoDouble slopTime("slopTime", &registry);
   variable::YoDouble cleanPosition("cleanPosition", &registry);
   variable::YoDouble noisyPosition("noisyPosition", &registry);
   variable::YoDouble cleanVelocity("cleanVelocity", &registry);

   variable::YoDouble reconstructedPosition2("reconstructedPosition2", &registry);
   variable::YoDouble totalReconstructedPositionError2("totalReconstructedPositionError2", &registry);
   variable::YoDouble averageReconstructedPositionError2("averageReconstructedPositionError2", &registry);

   double dt = 0.001;
   double totalTime = 5.0;

   double amplitude = 2.0;
   double frequency = 1.0;
   double noiseAmplitude = 0.01;
   std::uniform_real_distribution<double> noiseDist(-noiseAmplitude, noiseAmplitude);

   slopTime.set(0.1);
   alphaVariable.set(0.95);

   BacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity("bl_qd_velocity2", "", alphaVariable, dt, slopTime, &registry, &noisyPosition);

   reconstructedPosition2.set(amplitude);

   for (double time = 0.0; time < totalTime; time = time + dt)
   {
      cleanPosition.set(amplitude * std::cos(2.0 * M_PI * frequency * time));
      cleanVelocity.set(-2.0 * M_PI * amplitude * frequency * std::sin(2.0 * M_PI * frequency * time));

      noisyPosition.set(cleanPosition.getDoubleValue());
      noisyPosition.add(noiseDist(random));

      revisedBacklashCompensatingVelocity.update();

      reconstructedPosition2.add(revisedBacklashCompensatingVelocity.getDoubleValue() * dt);

      double positionError2 = reconstructedPosition2.getDoubleValue() - cleanPosition.getDoubleValue();
      totalReconstructedPositionError2.add(std::abs(positionError2) * dt);
   }

   averageReconstructedPositionError2.set(totalReconstructedPositionError2.getDoubleValue() / totalTime);

   // The original one doesn't do very well with noisy signals because it thinks the noise is backlash.
   EXPECT_LT(averageReconstructedPositionError2.getDoubleValue(), 0.25);
}

TEST(BacklashCompensatingVelocityYoVariableTest, testSignalWithBacklash)
{
   registry::YoRegistry registry("Registry");
   variable::YoDouble alphaVariable("alpha", &registry);
   variable::YoDouble slopTime("slopTime", &registry);

   variable::YoDouble cleanPosition("cleanPosition", &registry);
   variable::YoDouble backlashyPosition("backlashyPosition", &registry);
   variable::YoDouble cleanVelocity("cleanVelocity", &registry);

   variable::YoDouble reconstructedPosition2("reconstructedPosition2", &registry);
   variable::YoDouble totalReconstructedPositionError2("totalReconstructedPositionError2", &registry);
   variable::YoDouble averageReconstructedPositionError2("averageReconstructedPositionError2", &registry);

   double dt = 0.001;
   double totalTime = 5.0;

   double amplitude = 2.0;
   double frequency = 1.0;
   double backlashAmount = 0.1;

   slopTime.set(0.1);
   alphaVariable.set(0.95);

   BacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity("bl_qd_velocity2", "", alphaVariable, dt, slopTime, &registry,
                                                                               &backlashyPosition);

   reconstructedPosition2.set(amplitude);

   for (double time = 0.0; time < totalTime; time = time + dt)
   {
      cleanPosition.set(amplitude * std::cos(2.0 * M_PI * frequency * time));
      cleanVelocity.set(-2.0 * M_PI * amplitude * frequency * std::sin(2.0 * M_PI * frequency * time));

      backlashyPosition.set(cleanPosition.getDoubleValue());
      if (cleanVelocity.getDoubleValue() > 0.0)
         backlashyPosition.add(backlashAmount);

      revisedBacklashCompensatingVelocity.update();

      reconstructedPosition2.add(revisedBacklashCompensatingVelocity.getDoubleValue() * dt);

      double positionError2 = reconstructedPosition2.getDoubleValue() - cleanPosition.getDoubleValue();
      totalReconstructedPositionError2.add(std::abs(positionError2) * dt);
   }

   averageReconstructedPositionError2.set(totalReconstructedPositionError2.getDoubleValue() / totalTime);

   EXPECT_LT(averageReconstructedPositionError2.getDoubleValue(), 0.25);
}

TEST(BacklashCompensatingVelocityYoVariableTest, testRemoveSquareWaveBacklash)
{
   registry::YoRegistry registry("Registry");
   variable::YoDouble alphaVariable("alpha", &registry);
   variable::YoDouble slopTime("slopTime", &registry);

   variable::YoDouble backlashyPosition("backlashyPosition", &registry);

   double dt = 0.001;
   double totalTime = 5.0;

   double frequency = 30.0;
   double backlashAmount = 0.1;

   slopTime.set(0.1);
   alphaVariable.set(0.95);

   BacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity("bl_qd_velocity2", "", alphaVariable, dt, slopTime, &registry,
                                                                               &backlashyPosition);

   // Initialize the system to make sure it's resting up against one of the heads of slop. Previously
   // without this, it was a lucky test.
   backlashyPosition.set(0.0);
   revisedBacklashCompensatingVelocity.update();
   backlashyPosition.set(backlashAmount);
   for (int i = 0; i < static_cast<int>(2.0 * slopTime.getDoubleValue() / dt); i++)
      revisedBacklashCompensatingVelocity.update();

   for (double time = 0.0; time < totalTime; time = time + dt)
   {
      backlashyPosition.set(std::cos(2.0 * M_PI * frequency * time));
      if (backlashyPosition.getDoubleValue() > 0.0)
         backlashyPosition.set(backlashAmount);
      else
         backlashyPosition.set(-backlashAmount);

      revisedBacklashCompensatingVelocity.update();

      EXPECT_NEAR(0.0, revisedBacklashCompensatingVelocity.getDoubleValue(), 1e-3);
   }
}
} // namespace
} // namespace ihmc::yovariables::filters
