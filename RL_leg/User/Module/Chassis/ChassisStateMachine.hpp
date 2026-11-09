#pragma once

#include <stdint.h>

#define STATUS_MAX (10)

enum Enum_Chassis_States
{
    STOP = 0,
    EMERGENCY_STOP,
    CONTROL,
    STATUS_COUNT
};

enum class ChassisEvent
{
    None = 0,
    StopRequested,
    EmergencyStopRequested,
    ControlRequested,
};

struct Struct_Status
{
    const char* Name;
    uint32_t Enter_Count;
    uint32_t Total_Run_Time;
    void* User_Data;
};

class Chassis_FSM
{
public:
    Struct_Status Status[STATUS_MAX];
    Enum_Chassis_States State_chassis;

    void Init();
    inline Enum_Chassis_States Get_Now_State();
    inline const char* Get_Now_State_Name();
    void Dispatch(ChassisEvent event);
    void ForceStop();
    void TIM_Update();
    uint32_t Get_State_Run_Time(Enum_Chassis_States state);
    uint32_t Get_State_Enter_Count(Enum_Chassis_States state);
    void Reset_State_Statistics(Enum_Chassis_States state);

private:
    uint32_t State_Run_Time[STATUS_MAX] = {0};
    void TransitTo(Enum_Chassis_States next_state);
};

inline Enum_Chassis_States Chassis_FSM::Get_Now_State()
{
    return State_chassis;
}

inline const char* Chassis_FSM::Get_Now_State_Name()
{
    return Status[State_chassis].Name;
}
