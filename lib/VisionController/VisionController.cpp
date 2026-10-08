#include "VisionController.h"
#include "VisionConfig.h"

namespace VisionProtocol
{
void makeFillLightCommand(bool enabled, uint8_t command[LIGHT_COMMAND_SIZE])
{
    command[0] = VisionConfig::FILL_LIGHT_HEADER;
    command[1] = enabled ? VisionConfig::FILL_LIGHT_ON
                         : VisionConfig::FILL_LIGHT_OFF;
    command[2] = VisionConfig::FILL_LIGHT_TAIL;
}

bool decodeTargetFrame(const uint8_t bytes[TARGET_FRAME_SIZE], TargetFrame &frame)
{
    if (bytes == nullptr || bytes[0] != 0xAA || bytes[4] != 0xBB)
    {
        return false;
    }
    frame.offsetXpx = static_cast<int8_t>(bytes[1]);
    frame.offsetYpx = static_cast<int8_t>(bytes[2]);
    frame.classId = bytes[3];
    return true;
}
} // namespace VisionProtocol
