// Runnable example: a YoVariableServer publishing a handful of continuously-changing variables,
// for manual (or scripted) interop verification against the real Java YoVariableClient. See
// README.md's "Server Phase 4 done" section for how this was used to verify interop.
//
// Usage: ./simple_server_example [durationSeconds]  (default: runs until Ctrl+C)

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <thread>

#include "ihmc/robotDataLogger/data_server_settings.h"
#include "ihmc/robotDataLogger/yo_variable_server.h"
#include "ihmc/yovariables/registry/yo_registry.h"
#include "ihmc/yovariables/variable/yo_boolean.h"
#include "ihmc/yovariables/variable/yo_double.h"
#include "ihmc/yovariables/variable/yo_enum.h"
#include "ihmc/yovariables/variable/yo_integer.h"
#include "ihmc/yovariables/variable/yo_long.h"

using namespace ihmc::yovariables;

namespace
{
enum class ExampleColor
{
   RED,
   GREEN,
   BLUE
};

std::atomic<bool> keepRunning{true};

void handleSigint(int)
{
   keepRunning = false;
}
}

int main(int argc, char** argv)
{
   std::signal(SIGINT, handleSigint);

   double durationSeconds = argc > 1 ? std::stod(argv[1]) : -1.0; // -1 => run until Ctrl+C

   registry::YoRegistry mainRegistry("Main");
   variable::YoDouble sineWave("sineWave", &mainRegistry);
   variable::YoInteger counter("counter", &mainRegistry);
   variable::YoBoolean toggle("toggle", &mainRegistry);
   variable::YoLong bigCounter("bigCounter", &mainRegistry);
   variable::YoEnum<ExampleColor> color("color", &mainRegistry, false);

   ihmc::robotDataLogger::YoVariableServer server("CppExampleServer", ihmc::robotDataLogger::DataServerSettings(false, false, 8008), 0.01);
   server.setMainRegistry(&mainRegistry);
   server.start();

   std::printf("YoVariableServer running on port %u. Connect the Java YoVariableClient to localhost:8008.\n", server.port());
   std::printf("Publishing: sineWave (double), counter (int), toggle (bool), bigCounter (long), color (enum).\n");

   int iteration = 0;
   auto startTime = std::chrono::steady_clock::now();
   while (keepRunning)
   {
      double elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
      if (durationSeconds >= 0.0 && elapsedSeconds >= durationSeconds)
         break;

      sineWave.set(10.0 * std::sin(2.0 * M_PI * 0.5 * elapsedSeconds));
      counter.set(iteration);
      toggle.set(iteration % 2 == 0);
      bigCounter.set(static_cast<std::int64_t>(iteration) * 1000);
      color.set(iteration % 3);

      std::int64_t timestampNanos = static_cast<std::int64_t>(elapsedSeconds * 1e9);
      server.update(timestampNanos);

      if (iteration % 100 == 0)
         std::printf("[%.1fs] sineWave=%.3f counter=%d toggle=%d bigCounter=%lld color=%s\n", elapsedSeconds, sineWave.getDoubleValue(),
                     counter.getIntegerValue(), toggle.getBooleanValue(), static_cast<long long>(bigCounter.getLongValue()),
                     color.getEnumValuesAsString()[static_cast<std::size_t>(color.getOrdinal())].c_str());

      iteration++;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
   }

   std::printf("Shutting down.\n");
   server.close();
   return 0;
}
