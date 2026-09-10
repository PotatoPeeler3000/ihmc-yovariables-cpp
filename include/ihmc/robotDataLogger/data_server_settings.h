#pragma once

namespace ihmc::robotDataLogger
{
/**
 * Mirrors Java's `us.ihmc.robotDataLogger.logger.DataServerSettings`. `autoDiscoverable` and
 * `logSession` are accepted and stored (surfacing into the Announcement JSON where applicable) but
 * not otherwise implemented in this phase - UDP multicast autodiscovery and disk logging are
 * deferred scope (see the plan).
 */
struct DataServerSettings
{
   bool logSession = false;
   bool autoDiscoverable = true;
   unsigned short port = 8008;

   DataServerSettings() = default;
   explicit DataServerSettings(bool logSession_) : logSession(logSession_)
   {
   }
   DataServerSettings(bool logSession_, bool autoDiscoverable_) : logSession(logSession_), autoDiscoverable(autoDiscoverable_)
   {
   }
   DataServerSettings(bool logSession_, bool autoDiscoverable_, unsigned short port_)
      : logSession(logSession_), autoDiscoverable(autoDiscoverable_), port(port_)
   {
   }
};
}
