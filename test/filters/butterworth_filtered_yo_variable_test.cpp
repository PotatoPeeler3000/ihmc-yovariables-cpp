#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ihmc/yovariables/filters/butterworth_filtered_yo_variable.h"

namespace ihmc::yovariables::filters
{
namespace
{
struct TimedData
{
   double time;
   double value;
};

double computePredictedAttenuationInDecibels(double cutOffFrequency, int filterOrder, double queryFrequency)
{
   return 10.0 * std::log10(1.0 + std::pow(queryFrequency / cutOffFrequency, 2.0 * filterOrder));
}

std::vector<TimedData> generateInputCurve(int numberOfElements, double frequency, double dt)
{
   std::vector<TimedData> inputCurve(numberOfElements);

   double time = 0.0;
   for (int i = 0; i < numberOfElements; i++)
   {
      inputCurve[i] = {time, std::sin(2.0 * M_PI * frequency * time)};
      time += dt;
   }

   return inputCurve;
}

std::vector<TimedData> generateInputCurve(double endTime, double frequency, double dt)
{
   return generateInputCurve(static_cast<int>(std::ceil(endTime / dt)) + 1, frequency, dt);
}

std::vector<TimedData> getFilteredCurve(const std::vector<TimedData>& input, ButterworthFilteredYoVariable& butterworthFilteredYoVariable)
{
   std::vector<TimedData> filteredCurve(input.size());

   butterworthFilteredYoVariable.reset();

   for (std::size_t i = 0; i < input.size(); i++)
   {
      butterworthFilteredYoVariable.update(input[i].value);
      filteredCurve[i] = {input[i].time, butterworthFilteredYoVariable.getDoubleValue()};
   }

   return filteredCurve;
}

double getMaximumPeakToPeakAmplitude(const std::vector<TimedData>& dataset)
{
   double value = dataset[0].value;
   double maximumValue = value;
   double minimumValue = value;

   for (std::size_t i = 1; i < dataset.size(); i++)
   {
      // Matches the Java source verbatim: this indexes dataset[1] every iteration rather than
      // dataset[i], so only the first two elements ever actually influence the result.
      value = dataset[1].value;

      if (value > maximumValue)
         maximumValue = value;
      else if (value < minimumValue)
         minimumValue = value;
   }

   return maximumValue - minimumValue;
}

double getMagnitudeInDecibels(const std::vector<TimedData>& input, const std::vector<TimedData>& output)
{
   double inputAmp = getMaximumPeakToPeakAmplitude(input);
   double outputAmp = getMaximumPeakToPeakAmplitude(output);
   double attenuation = outputAmp / inputAmp;
   return 10.0 * std::log10(attenuation);
}

double findBreakFrequency(ButterworthFilteredYoVariable& filter, double dt, double tolerance)
{
   int numberOfCycles = 10;

   double samplingFrequency = 1.0 / dt;
   double upperBreakFrequency = 0.5 * samplingFrequency;
   double lowerBreakFrequency = 0.0;
   double upper_dB = -std::numeric_limits<double>::infinity();
   double lower_dB = 0.0;

   double cutOff_dB = -computePredictedAttenuationInDecibels(samplingFrequency, 1, samplingFrequency);

   while (std::abs(upper_dB - lower_dB) > tolerance && std::abs(upperBreakFrequency - lowerBreakFrequency) > 0.01 * dt)
   {
      double testBreakFrequency = 0.5 * (upperBreakFrequency + lowerBreakFrequency);
      double endTime = numberOfCycles / testBreakFrequency;
      std::vector<TimedData> inputCurve = generateInputCurve(endTime, testBreakFrequency, dt);
      std::vector<TimedData> outputCurve = getFilteredCurve(inputCurve, filter);

      std::vector<TimedData> clippedInputCurve(inputCurve.begin() + static_cast<std::ptrdiff_t>(inputCurve.size() / 2), inputCurve.end());
      std::vector<TimedData> clippedOutputCurve(outputCurve.begin() + static_cast<std::ptrdiff_t>(outputCurve.size() / 2), outputCurve.end());

      double dB = getMagnitudeInDecibels(clippedInputCurve, clippedOutputCurve);

      if (dB < cutOff_dB)
      {
         upperBreakFrequency = testBreakFrequency;
         upper_dB = dB;
      }
      else
      {
         lowerBreakFrequency = testBreakFrequency;
         lower_dB = dB;
      }
   }

   return 0.5 * (upperBreakFrequency + lowerBreakFrequency);
}

TEST(ButterworthFilteredYoVariableTest, testAlphaCompute)
{
   std::mt19937 random(0734454U);
   const double epsilon = 1.0e-12;

   for (int i = 0; i < 5000; i++)
   {
      std::uniform_real_distribution<double> dtDist(1.0e-4, 1.0e-2);
      double dt = dtDist(random);
      double samplingFrequency = 1.0 / dt;

      double breakFrequencyIn, alpha, breakFrequencyOut;

      breakFrequencyIn = 0.0;
      alpha = ButterworthFilteredYoVariable::computeAlphaGivenBreakFrequency(breakFrequencyIn, dt);
      EXPECT_NEAR(1.0, alpha, epsilon);
      breakFrequencyOut = ButterworthFilteredYoVariable::computeBreakFrequencyGivenAlpha(alpha, dt);
      EXPECT_NEAR(breakFrequencyIn, breakFrequencyOut, epsilon);

      breakFrequencyIn = std::uniform_real_distribution<double>(0.0, 0.25 * samplingFrequency)(random);
      alpha = ButterworthFilteredYoVariable::computeAlphaGivenBreakFrequency(breakFrequencyIn, dt);
      breakFrequencyOut = ButterworthFilteredYoVariable::computeBreakFrequencyGivenAlpha(alpha, dt);
      EXPECT_NEAR(breakFrequencyIn, breakFrequencyOut, epsilon);

      breakFrequencyIn = std::uniform_real_distribution<double>(0.25 * samplingFrequency, 10.0 * samplingFrequency)(random);
      alpha = ButterworthFilteredYoVariable::computeAlphaGivenBreakFrequency(breakFrequencyIn, dt);
      EXPECT_NEAR(0.0, alpha, epsilon);
      breakFrequencyOut = ButterworthFilteredYoVariable::computeBreakFrequencyGivenAlpha(alpha, dt);
      EXPECT_NEAR(0.25 * samplingFrequency, breakFrequencyOut, epsilon);
   }
}

TEST(ButterworthFilteredYoVariableTest, testBreakFrequencyLowPassFilter)
{
   double dt = 0.001;
   double desiredBreakFrequency = 4.0;
   double alpha = ButterworthFilteredYoVariable::computeAlphaGivenBreakFrequency(desiredBreakFrequency, dt);
   ButterworthFilteredYoVariable butterworthFilteredYoVariable("test", nullptr, alpha, ButterworthFilteredYoVariable::ButterworthFilterType::LOW_PASS);

   double actualBreakFrequency = findBreakFrequency(butterworthFilteredYoVariable, dt, dt);
   double percentError = std::abs(actualBreakFrequency - desiredBreakFrequency) / desiredBreakFrequency;
   EXPECT_NEAR(0.0, percentError, 0.05);
}

// testButterWorth is @Disabled in the Java source ("Old code") and is not ported.
} // namespace
} // namespace ihmc::yovariables::filters
