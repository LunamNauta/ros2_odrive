#ifndef ODRIVE_CAN_HEADER
#define ODRIVE_CAN_HEADER

#include <cstdint>
#include <climits>
#include <vector>
#include <string>

#include <linux/can.h>

namespace ODrive_CAN{

typedef std::int16_t Int16_t;
typedef std::int32_t Int32_t;

typedef std::uint8_t Uint8_t;
typedef std::uint16_t Uint16_t;
typedef std::uint32_t Uint32_t;

typedef float Float32_t;

static_assert(CHAR_BIT == 8, "Error: Byte is not 8-bit");        // This is unnecessary
static_assert(sizeof(float) == 4, "Error: Float is not 32-bit"); // This is incomplete. Does not check for IEEE-754 format

enum class COMMAND{
    HEARTBEAT                = 0x001,
    GET_MOTOR_ERROR          = 0x003,
    GET_ENCODER_ERROR        = 0x004,
    GET_SENSORLESS_ERROR     = 0x005,
    SET_AXIS_NODE_ID         = 0x006,
    SET_AXIS_STATE           = 0x007,
    GET_ENCODER_ESTIMATES    = 0x009,
    GET_ENCODER_COUNT        = 0x00a,
    SET_CONTROLLER_MODE      = 0x00b,
    SET_INPUT_POS            = 0x00c,
    SET_INPUT_VEL            = 0x00d,
    SET_INPUT_TORQUE         = 0x00e,
    SET_LIMITS               = 0x00f,
    START_ANTICOGGING        = 0x010,
    SET_TRAJ_VEL_LIMIT       = 0x011,
    SET_TRAJ_ACCEL_LIMITS    = 0x012,
    SET_TRAJ_INTERTIA        = 0x013,
    GET_IQ                   = 0x014,
    GET_SENSORLESS_ESTIMATES = 0x015,
    REBOOT                   = 0x016,
    GET_BUS_VOLTAGE_COUNT    = 0x017,
    CLEAR_ERRORS             = 0x018,
    SET_LINEAR_COUNT         = 0x019,
    SET_POS_GAIN             = 0x01a,
    SET_VEL_GAINS            = 0x01b,
    GET_ADC_VOLTAGE          = 0x01c,
    GET_CONTROLLER_ERROR     = 0x01d,
    NONE
};

struct Heartbeat{
    Uint32_t axis_error;
    Uint8_t axis_state;
    Uint8_t motor_error_flag;
    Uint8_t encoder_error_flag;
    Uint8_t controller_error_flag;
    Uint8_t trajectory_done_flag;
} __attribute__((packed));
struct Get_Motor_Error{
    Uint32_t motor_error;
} __attribute__((packed));
struct Get_Encoder_Error{
    Uint32_t encoder_error;
} __attribute__((packed));
struct Get_Sensorless_Error{
    Uint32_t sensorless_error;
} __attribute__((packed));
struct Set_Axis_Node_ID{
    Uint32_t axis_node_id;
} __attribute__((packed));
struct Set_Axis_State{
    Uint32_t axis_requested_state;
} __attribute__((packed));
struct Get_Encoder_Estimates{
    Uint32_t pos_estimate;
    Uint32_t vel_estimate;
} __attribute__((packed));
struct Get_Encoder_Count{
    Uint32_t shadow_count;
    Uint32_t count_in_cpr;
} __attribute__((packed));
struct Set_Controller_Mode{
    Uint32_t control_mode;
    Uint32_t input_mode;
} __attribute__((packed));
struct Set_Input_Pos{
    Uint32_t input_pos;
    Int16_t vel_ff;
    Int16_t torque_ff;
} __attribute__((packed));
struct Set_Input_Vel{
    Uint32_t input_vel;
    Uint32_t input_torque_ff;
} __attribute__((packed));
struct Set_Input_Torque{
    Uint32_t input_torque;
} __attribute__((packed));
struct Set_Limits{
    Uint32_t velocity_limit;
    Uint32_t current_limit;
} __attribute__((packed));
struct Start_Anticogging{};
struct Set_Traj_Vel_Limit{
    Uint32_t traj_vel_limit;
} __attribute__((packed));
struct Set_Traj_Accel_Limits{
    Uint32_t traj_accel_limit;
    Uint32_t traj_decel_limit;
} __attribute__((packed));
struct Set_Traj_Inertia{
    Uint32_t traj_inertia;
} __attribute__((packed));
struct Get_Iq{
    Uint32_t iq_setpoint;
    Uint32_t iq_measured;
} __attribute__((packed));
struct Get_Sensorless_Estimates{
    Uint32_t sensorless_pos_estimate;
    Uint32_t sensorless_vel_estimate;
} __attribute__((packed));
struct Reboot{} __attribute__((packed));
struct Get_Bus_Voltage_Current{
    Uint32_t bus_voltage;
    Uint32_t bus_current;
} __attribute__((packed));
struct Clear_Errors{} __attribute__((packed));
struct Set_Linear_Count{
    Int32_t position;
} __attribute__((packed));
struct Set_Pos_Gain{
    Uint32_t pos_gain;
} __attribute__((packed));
struct Set_Vel_Gains{
    Uint32_t vel_gain;
    Uint32_t vel_integrator_gain;
} __attribute__((packed));
struct Get_ADC_Voltage{
    Uint32_t adc_voltage;
} __attribute__((packed));
struct Get_Controller_Error{
    Uint32_t controller_error;
} __attribute__((packed));

struct CAN_Frame{
    Uint8_t node_id;
    Uint8_t cmd_id;
    Uint8_t len;
    union{
        Heartbeat heartbeat;
        Get_Motor_Error get_motor_error;
        Get_Encoder_Error get_encoder_error;
        Get_Sensorless_Error get_sensorless_error;
        Set_Axis_Node_ID set_axis_node_id;
        Set_Axis_State set_axis_state;
        Get_Encoder_Estimates get_encoder_estimates;
        Get_Encoder_Count get_encoder_count;
        Set_Controller_Mode set_controller_mode;
        Set_Input_Pos set_input_pos;
        Set_Input_Vel set_input_vel;
        Set_Input_Torque set_input_torque;
        Set_Limits set_limits;
        Start_Anticogging start_anticogging;
        Set_Traj_Vel_Limit set_traj_vel_limit;
        Set_Traj_Accel_Limits set_traj_accel_limits;
        Set_Traj_Inertia set_traj_inertia;
        Get_Iq get_iq;
        Get_Sensorless_Estimates get_sensorless_estimates;
        Reboot reboot;
        Get_Bus_Voltage_Current get_bus_voltage_current;
        Clear_Errors clear_errors;
        Set_Linear_Count set_linear_count;
        Set_Pos_Gain set_pos_gain;
        Set_Vel_Gains set_vel_gains;
        Get_ADC_Voltage get_adc_voltage;
        Get_Controller_Error get_controller_error;
    } cmd;

    CAN_Frame() : node_id(-1), cmd_id((Uint8_t)COMMAND::NONE){}
};

void Clear_CAN_Bus(int can_socket);

CAN_Frame Read_CAN_Bus1(int can_socket);
std::vector<CAN_Frame> Read_CAN_Bus(int can_socket, std::size_t max = 1);

ssize_t Write_CAN_Bus1(int can_socket, const CAN_Frame& frame);
std::vector<ssize_t> Write_Can_Bus(int can_socket, const std::vector<CAN_Frame>& frames);

int Create_CAN_Socket();
int Bind_CAN_Socket(int can_socket, const std::string& can_id, sockaddr_can* out_addr = nullptr);
int Close_CAN_Socket(int can_socket);

}

#endif
