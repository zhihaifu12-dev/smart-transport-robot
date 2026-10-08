#ifndef REWIND_DIAGNOSTICS_H
#define REWIND_DIAGNOSTICS_H

#include <Arduino.h>

namespace Diagnostics
{
enum class FaultCode : uint8_t
{
    None = 0,
    ArmInitializationFailed,
    RawVisionTimeout,
    DigitVisionTimeout,
    StorageTrayOffline,
    InvalidTaskCode
};

void logState(const char *component, const char *state);
void logFault(FaultCode code, const char *detail);
} // namespace Diagnostics

#endif
