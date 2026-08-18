#ifndef REWIND_ARM_CONTROLLER_H
#define REWIND_ARM_CONTROLLER_H

#include <Arduino.h>

class ArmController
{
public:
    bool begin();

    // M5机械臂底座步进电机，单位：0.1 degree。
    void moveBaseTo(float angleTenthDegree);

    // M6水平伸缩轴，单位：0.1 mm。
    void moveHorizontalTo(float positionTenthMm);

    // M7垂直升降轴，单位：0.1 mm。
    void moveVerticalTo(float positionTenthMm);

    void setStowed(bool stowed);
    void beginAreaReset(bool rawArea);
    void service();
    bool resetComplete() const;
    bool prepareDigitAreaOnArrival();

    bool pickRawBatch(const uint8_t colors[3], const uint8_t slots[3],
                      bool secondRoundArrival = false);
    bool processDigitArea(const uint8_t sourceSlots[3],
                          const uint8_t targetDigits[3],
                          bool pickBack,
                          bool reuseTemporaryCoordinates = false,
                          float placeDepthReductionTenthMm = 0.0f,
                          const uint8_t targetColors[3] = nullptr);
};

#endif
