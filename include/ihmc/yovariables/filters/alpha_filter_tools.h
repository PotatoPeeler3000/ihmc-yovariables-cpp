#pragma once

namespace ihmc::yovariables::filters
{
/**
 * Computes the alpha for an alpha-filtered variable equivalent to a first-order low-pass filter
 * at breakFrequencyInHertz. For a repeated computation as part of a DoubleProvider, prefer
 * AlphaBasedOnBreakFrequencyProvider, which only recomputes on a change in break frequency.
 */
double computeAlphaGivenBreakFrequencyProperly(double breakFrequencyInHertz, double dt);

/** The break frequency of a first-order low-pass filter given an alpha value. */
double computeBreakFrequencyGivenAlpha(double alpha, double dt);
}
