#pragma once

#include <string>

#include "ihmc/robotDataLogger/handshake/handshake_types.h"

namespace ihmc::robotDataLogger::handshake
{
/**
 * Hand-written JSON serialization matching the exact shape Java's `ROS2JSONSerializer`/
 * `CDRInterchangeSerializer` produce for `Handshake`/`Announcement` (root-wrapped under
 * `"us::ihmc::robotDataLogger::&lt;TypeName&gt;"`, camelCase fields, `YoType`/`loadStatus` as enum-name
 * strings) - this is what the Java `YoVariableClient` actually parses from `GET /handshake.json` and
 * `GET /announcement.json`, so the shape must match exactly. Write-only: this library never needs to
 * parse JSON, since it only ever plays the server role.
 */
std::string toJsonString(const Handshake& handshake);
std::string toJsonString(const Announcement& announcement);
}
