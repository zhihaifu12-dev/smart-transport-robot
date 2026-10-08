#include "Diagnostics.h"

namespace Diagnostics
{
void logState(const char *component, const char *state)
{
    Serial.print("[STATE][");
    Serial.print(component);
    Serial.print("] ");
    Serial.println(state);
}

void logFault(FaultCode code, const char *detail)
{
    Serial.print("[FAULT][");
    Serial.print(static_cast<uint8_t>(code));
    Serial.print("] ");
    Serial.println(detail == nullptr ? "" : detail);
}
} // namespace Diagnostics
