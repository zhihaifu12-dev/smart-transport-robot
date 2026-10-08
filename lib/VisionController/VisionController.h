#ifndef REWIND_VISION_CONTROLLER_H
#define REWIND_VISION_CONTROLLER_H

#include <Arduino.h>

namespace VisionProtocol
{
constexpr size_t LIGHT_COMMAND_SIZE = 3;
constexpr size_t TARGET_FRAME_SIZE = 5;

struct TargetFrame
{
    int8_t offsetXpx = 0;
    int8_t offsetYpx = 0;
    uint8_t classId = 0;
};

void makeFillLightCommand(bool enabled, uint8_t command[LIGHT_COMMAND_SIZE]);
bool decodeTargetFrame(const uint8_t bytes[TARGET_FRAME_SIZE], TargetFrame &frame);
} // namespace VisionProtocol

#endif
