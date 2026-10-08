#include "PID.h"
PID Rot_PID;
PID MoveX_PID;
PID MoveY_PID;
PID LocalX_PID;
PID LocalY_PID;

PID ARM_PID;
PID ARM_BASE_PID;
PID FollowX_PID;
PID FollowY_PID;
// 用于清空PID
void PID::PID_Init(){
    error = 0;
    lastError = 0;
    integral = 0;
    output = 0;
    lastDerivative = 0;
}
// 用于初始化任意一个PID控制器
void PID::PID_Init(float p, float i, float d, float maxI, float maxOut)
{
    Kp = p;
    Ki = i;
    Kd = d;
    maxIntegral = maxI;
    maxOutput = maxOut;
    error = 0;
    lastError = 0;
    integral = 0;
    output = 0;
    lastDerivative = 0;
}
// 用于PID控制，输入参数为PID控制器、目标量与实际量
void PID::PID_Calc(float Target_Para, float Current_Para)
{
    // 更新数据
    lastError = error;             // 将旧error存起来
    error = Target_Para - Current_Para; // 计算新error
    if (this == &Rot_PID)
    {
        // 将任意圈数的误差统一映射到[-PI, PI]，始终沿最短方向旋转。
        while (error > M_PI)
        {
            error -= 2.0f * M_PI;
        }
        while (error < -M_PI)
        {
            error += 2.0f * M_PI;
        }
        // 计算积分
        if (fabs(error) < 0.5) // 0.5rad ≈ 28.6°
        {
            integral += (error) * (Ki);
            // 积分限幅
            integral = constrain(integral, -maxIntegral, maxIntegral);
        }
        else
        {
            integral = 0; // 大误差时重置积分
        }
    }else{
        // 计算积分
        if (fabs(error) < 500)
        {
            integral += (error) * (Ki);
            // 积分限幅
            integral = constrain(integral, -maxIntegral, maxIntegral);
        }
        else
        {
            integral = 0; // 大误差时重置积分
        }
    }
    // 计算微分
    float derivative = (error - lastError);
    float filteredDerivative = 0.7 * lastDerivative + 0.3 * derivative;
    lastDerivative = filteredDerivative;
    // 计算比例
    float P_out = (error) * (Kp);
    // if(this == &ARM_PID)
    // {
    //     //P_out = (error) * (Kp/2 + log10(fabs(error / 100) + 1));
    // }
    // 计算输出
    output = P_out + (Kd) * filteredDerivative + integral;
    // 输出限幅
    output = constrain(output, -maxOutput, maxOutput);
}


/// @brief // 移动机器人pid参数归零
void Car_PID_Init()
{
    MoveX_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    MoveY_PID.PID_Init(Move_PID_Kp, Move_PID_Ki, Move_PID_Kd, Move_PID_MItg, Move_PID_MOut);
    Rot_PID.PID_Init(Rot_PID_Kp, Rot_PID_Ki, Rot_PID_Kd, Rot_PID_MItg, Rot_PID_MOut);
}
