#ifndef REWIND_ROUTE_CONFIG_H
#define REWIND_ROUTE_CONFIG_H

#include <Arduino.h>
#include "MoveByPosition.h"

namespace RouteConfig
{
// 单位：ms。区分节点业务停留时间，不包含电机实际运动和机械稳定等待。
constexpr uint32_t START_DELAY_MS = 500;
constexpr uint32_t NORMAL_WAIT_MS = 300;

// [实机必调] 单位：mm。作用：启停区、回程二维码和最终停车坐标。
// 风险：坐标超出灰色车道会直接导致比赛结束。
constexpr float START_ZONE_X = 2220.0f;
constexpr float START_ZONE_1_Y = 2175.0f;
constexpr float START_ZONE_2_Y = 155.0f;
constexpr float RETURN_QR_ZONE_1_X = 2250.0f;
constexpr float RETURN_QR_ZONE_1_Y = 1200.0f;
constexpr float RETURN_QR_ZONE_2_X = 2150.0f;
constexpr float RETURN_QR_ZONE_2_Y = 1200.0f;
constexpr float RETURN_ZONE_1_X = 2220.0f;
constexpr float RETURN_ZONE_1_Y = 2245.0f;
constexpr float RETURN_ZONE_2_X = 2250.0f;
constexpr float RETURN_ZONE_2_Y = 150.0f;

// 单位：rad。场地坐标系：0=+X，PI/2=+Y，PI=-X，-PI/2=-Y。
constexpr float TRANSIT_HEADING_RAD = PI / 2.0f;
constexpr float RAW_WORK_HEADING_RAD = PI;
constexpr float COARSE_WORK_HEADING_RAD = 0.0f;
constexpr float TEMP_WORK_HEADING_RAD = -PI / 2.0f;

struct RouteNode
{
    RoutePointId point;
    float heading;
};

constexpr RouteNode ROUTE[] = {
    // 去程：二维码区离开后仍经过中心点，行驶中完成扫码，再进入首轮原料区。
    {POINT_QR, TRANSIT_HEADING_RAD},
    {POINT_CENTER, TRANSIT_HEADING_RAD},
    {POINT_RAW, RAW_WORK_HEADING_RAD},

    // 第一轮：先走到(暂存区X, 粗加工区Y)，再转向暂存区，形成直角折线。
    {POINT_COARSE, COARSE_WORK_HEADING_RAD},
    {POINT_COARSE_TEMP_CORNER, TRANSIT_HEADING_RAD},
    {POINT_TEMPORARY, TEMP_WORK_HEADING_RAD},

    // 暂存区 -> 第二轮原料区：经(暂存区X, 原料区Y)形成直角折线。
    {POINT_TEMP_RAW_CORNER, TRANSIT_HEADING_RAD},
    {POINT_RAW, RAW_WORK_HEADING_RAD},

    // 第二轮粗加工区 -> 暂存区沿用相同拐点和折线路线。
    {POINT_COARSE, COARSE_WORK_HEADING_RAD},
    {POINT_COARSE_TEMP_CORNER, TRANSIT_HEADING_RAD},
    {POINT_TEMPORARY, TEMP_WORK_HEADING_RAD},

    // 第二轮暂存区作业结束后，必须经过地图中心，再前往返程二维码区。
    {POINT_CENTER, TRANSIT_HEADING_RAD},
    {POINT_RETURN_QR, TRANSIT_HEADING_RAD},
    {POINT_FINISH, TRANSIT_HEADING_RAD}};

constexpr size_t ROUTE_COUNT = sizeof(ROUTE) / sizeof(ROUTE[0]);

// 第二轮地图从“第一轮暂存区 -> 第二轮原料区”的拐点开始使用，
// 覆盖第二轮原料区、粗加工区、暂存区及其两个折线拐点。
constexpr size_t SECOND_ROUND_MAP_BEGIN_ROUTE_INDEX = 6;
constexpr size_t SECOND_ROUND_MAP_END_ROUTE_INDEX = 10;
} // namespace RouteConfig

#endif
